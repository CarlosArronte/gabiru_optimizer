import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
import json
import numpy as np
from gabiru_optimizer.pso_optimizer import optimize_segment
from gabiru_shared.csv_utils import save_pp_params
import os
import csv
from std_srvs.srv import Trigger

CSV_PATH = "optimized_PP_parms.csv"

class PSOOptimizerNode(Node):
    def __init__(self):
        super().__init__('pso_optimizer_node')

        self.srv = self.create_service(Trigger,'ready_to_recive_optimization',self.ready_callback)#1
        self.client = self.create_client(Trigger, 'start_optimization')      
        self.publisher = self.create_publisher(String, 'gabiru/params', 10)
        self.optimized_segments = set()

        self.optim_msg = String()
    
    def ready_callback(self,request, response):
        self.subscription = self.create_subscription(#2
                String,
                'gabiru/segments',
                self.segment_callback,
                10
            )
        response.success = True
        response.message = "PathOptimizer listo para recibir datos"
        self.get_logger().info("Servicio ready_to_optimize: Suscripción a /processed_data activada")
        return response 
    
    def segment_callback(self, msg):
        self.get_logger().info(f"Mensaje recibido: {msg.data}")

        try:
            data = json.loads(msg.data)
            segment_id = data["segment_id"]
            tipo = data["tipo"]
            waypoints = data["waypoints"]
        except Exception as e:
            self.get_logger().error(f"Error al parsear JSON: {e}")
            return

        if segment_id in self.optimized_segments:
            self.get_logger().info(f"Segmento {segment_id} ya optimizado, ignorando.")
            return

        self.get_logger().info(f"Optimizando segmento {segment_id} tipo {tipo} con {len(waypoints)} waypoints...")

        try:
            waypoints_np = np.array(waypoints)  # <-- CONVERTIR A NUMPY ARRAY
            resultado = optimize_segment(tipo, waypoints_np)  # <-- PASAR TIPO Y WAYPOINTS
        except Exception as e:
            self.get_logger().error(f"Error en la optimización: {e}")
            return

        if resultado:

            best_pos, best_cost = resultado
            Ld, vd, w_max = best_pos

            #Se guardan en un csv
            self.save_to_csv(segment_id, Ld, vd, w_max)


            self.optim_msg.data = json.dumps({
                "segment_id": segment_id,
                "Ld": Ld,
                "vd": vd,
                "w_max": w_max
            })

            self.wait_for_publish()  
            
            
        else:
            self.get_logger().warning(f"No se pudo optimizar el segmento {segment_id}.")

    def wait_for_publish(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Esperando por modulo PP")
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.ready_response_callback)    
    

    def ready_response_callback(self,future):
        try:
            response = future.result()
            if response.success:
                optim_dict = json.loads(self.optim_msg.data)
                self.get_logger().info(f"PathProcessor listo: {response.message}")
                self.publisher.publish(self.optim_msg)
                self.get_logger().info(f"Publicado parámetros para segmento {optim_dict['segment_id']}: {self.optim_msg.data}")
                self.optimized_segments.add(optim_dict['segment_id'])
            else:
                self.get_logger().error(f"Fallo en la preparación: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Error al llamar al servicio: {e}")


    def save_to_csv(self,segment_id, Ld, vd, w_max):
        save_pp_params(segment_id, Ld, vd, w_max)

 

def main(args=None):
    rclpy.init(args=args)
    node = PSOOptimizerNode()
    node.get_logger().info("Nodo PSO iniciado")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
