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
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
            depth=1000
        )
        self.subscription = self.create_subscription(
            String,
            'gabiru/segments',
            self.segment_callback,
            qos)
        self.publisher = self.create_publisher(String, 'gabiru/params', qos)
        self.optimized_segments = set()
    
    def save_to_csv(self,segment_id, Ld, vd, w_max):
        save_pp_params(segment_id, Ld, vd, w_max)

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


            msg_out = String()
            msg_out.data = json.dumps({
                "segment_id": segment_id,
                "Ld": Ld,
                "vd": vd,
                "w_max": w_max
            })
            self.publisher.publish(msg_out)
            self.get_logger().info(f"Publicado parámetros para segmento {segment_id}: {msg_out.data}")
            self.optimized_segments.add(segment_id)
        else:
            self.get_logger().warning(f"No se pudo optimizar el segmento {segment_id}.")

def main(args=None):
    rclpy.init(args=args)
    node = PSOOptimizerNode()
    node.get_logger().info("Nodo PSO iniciado")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
