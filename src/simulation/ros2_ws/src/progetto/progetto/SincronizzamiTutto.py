import concurrent.futures
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup

import uuid
import json
import threading

class SincronizzamiTutto:
    def __init__(self, nodo, topic_richiesta, topic_risposta, timeout):
        self.nodo = nodo
        self.timeout = timeout
        self.lock = threading.Lock()
        self.pending = {}
        self.cb_group = ReentrantCallbackGroup()
        self.pub = nodo.create_publisher(String, topic_richiesta, 10)
        self.sub = nodo.create_subscription(String, topic_risposta,
            self.on_response, 10, callback_group=self.cb_group)

    def on_response(self, msg):
        try:
            packet = json.loads(msg.data)
        except:
            self.nodo.get_logger().info(f"Messaggio non JSON: {msg.data}")
            return
        req_id = packet.get("id")
        with self.lock:
            future = self.pending.pop(req_id, None)
        if future:
            if packet.get("success", True):
                future.set_result(packet.get('payload'))
            else:
                err_msg = packet.get("error", 'Errore generico')
                future.set_exception(Exception(err_msg))

    def call(self, payload):
        req_id = uuid.uuid4().hex
        self.nodo.get_logger().info(f"[DEBUG 1] Preparo richiesta ID: {req_id}")
        fut = concurrent.futures.Future()
        with self.lock:
            self.pending[req_id] = fut
        ros_msg = String()
        ros_msg.data = json.dumps({"id": req_id, "payload": payload})
        self.nodo.get_logger().info(f"[DEBUG 2] Sto pubblicando su {self.pub.topic_name}...")
        self.pub.publish(ros_msg)
        self.nodo.get_logger().info(f"[DEBUG 3] Pubblicato! Ora aspetto la risposta...")
        try:
            risultato = fut.result(timeout=self.timeout)
            self.nodo.get_logger().info(f"[DEBUG 4] Risposta ricevuta!")
            return risultato
        except concurrent.futures.TimeoutError:
            self.nodo.get_logger().error(f"[DEBUG ERROR] Timeout scaduto per ID {req_id}")
            with self.lock:
                self.pending.pop(req_id, None)
            raise Exception(f"Timeout ({self.timeout}s)! Nessuna risposta ricevuta.")
