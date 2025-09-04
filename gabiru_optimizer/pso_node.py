import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
import csv
import os
from std_srvs.srv import Trigger
from gabiru_optimizer.pso_optimizer import optimize_segment

CSV_PATH = os.path.expanduser("~/sim_ws/optimized_PP_params.csv")

def save_pp_params(segment_id, tipo, Ld, vd, w_max, start_idx, end_idx, csv_path=CSV_PATH):
    """
    Guarda los parámetros optimizados en un archivo CSV.
    Si el archivo no existe, crea uno nuevo con encabezados.
    """
    file_exists = os.path.isfile(csv_path)
    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['segment_id', 'tipo', 'Ld', 'vd', 'w_max', 'start_idx', 'end_idx'])
        if not file_exists:
            writer.writeheader()
        writer.writerow({
            'segment_id': segment_id,
            'tipo': tipo,
            'Ld': Ld,
            'vd': vd,
            'w_max': w_max,
            'start_idx': start_idx,
            'end_idx': end_idx
        })

class PSOOptimizerNode(Node):
    def __init__(self):
        super().__init__('pso_optimizer_node')
        self.srv = self.create_service(Trigger, 'ready_to_receive_optimization', self.ready_callback)
        self.optimized_segments = set()
        self.get_logger().info("Nodo PSO iniciado")

    def ready_callback(self, request, response):
        self.subscription = self.create_subscription(
            String,
            'gabiru/segments',
            self.segment_callback,
            10
        )
        response.success = True
        response.message = "PSOOptimizer listo para recibir datos"
        self.get_logger().info("Servicio ready_to_receive_optimization: Suscripción a /gabiru/segments activada")
        return response

    def segment_callback(self, msg):
        self.get_logger().info(f"Mensaje recibido: {msg.data}")
        try:
            data = json.loads(msg.data)
            segment_id = data["segment_id"]
            tipo = data["tipo"]
            waypoints = data["waypoints"]
            indices = data["indices"]  # Ahora incluido en el JSON
        except Exception as e:
            self.get_logger().error(f"Error al parsear JSON: {e}")
            return

        if segment_id in self.optimized_segments:
            self.get_logger().info(f"Segmento {segment_id} ya optimizado, ignorando.")
            return

        self.get_logger().info(f"Optimizando segmento {segment_id} tipo {tipo} con {len(waypoints)} waypoints...")

        try:
            waypoints_np = np.array(waypoints)
            resultado = optimize_segment(tipo, waypoints_np)
        except Exception as e:
            self.get_logger().error(f"Error en la optimización: {e}")
            return

        if resultado:
            best_pos, best_cost = resultado
            Ld, vd, w_max = best_pos
            start_idx = indices[0]
            end_idx = indices[-1]
            save_pp_params(segment_id, tipo, Ld, vd, w_max, start_idx, end_idx)
            self.get_logger().info(f"Parámetros guardados para segmento {segment_id}: Ld={Ld}, vd={vd}, w_max={w_max}, start_idx={start_idx}, end_idx={end_idx}")
            self.optimized_segments.add(segment_id)
        else:
            self.get_logger().warning(f"No se pudo optimizar el segmento {segment_id}.")

def main(args=None):
    rclpy.init(args=args)
    node = PSOOptimizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()