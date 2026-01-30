import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import os
import logging

from neo4j import GraphDatabase
from neo4j.time import DateTime, Date, Duration, Time

from dotenv import load_dotenv
load_dotenv()

class Neo4jEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, (DateTime, Date, Duration, Time)):
            return str(obj)
        return super().default(obj)

class ServerNeo4j(Node):
    def __init__(self):
        super().__init__("ServerNeo4j")
        logging.getLogger("neo4j").setLevel(logging.ERROR)
        self.driver = None
        self.init_neo4j()
        self.sub = self.create_subscription(String, "/neo4j/request", self.handle_request, 10)
        self.pub = self.create_publisher(String, "/neo4j/response", 10)
        self.get_logger().info("ServerNeo4j avviato.")

    def init_neo4j(self):
        uri = os.getenv("NEO4J_URI")
        auth = (os.getenv("NEO4J_USER"), os.getenv("NEO4J_PASS"))
        try:
            self.driver = GraphDatabase.driver(uri, auth=auth)
            self.driver.verify_connectivity()
            self.get_logger().info("Connesso al DB Neo4j.")
        except Exception as e:
            self.get_logger().info(f"Errore connessione Neo4j: {e}")

    def handle_request(self, msg):
        # self.get_logger().info(f"[DEBUG 0] RICEVUTA RICHIESTA: {msg.data}")
        try:
            req = json.loads(msg.data)
            req_id = req.get('id')
            data = req.get('payload')
            query = data.get('query')
            params = data.get('params', {})
            result_list = []
            success = True
            error_msg = ""
            if self.driver:
                try:
                    with self.driver.session() as session:
                        res = session.run(query, params)
                        result_list = [r.data() for r in res]
                except Exception as e:
                    success = False
                    error_msg = str(e)
            else:
                success = False
                error_msg = "Driver Neo4j non connesso"
            res_packet = {
                "id": req_id,
                "success": success,
                "payload": result_list,
                "error": error_msg
            }
            out_msg = String()
            out_msg.data = json.dumps(res_packet, cls=Neo4jEncoder)
            self.pub.publish(out_msg)
        except Exception as e:
            self.get_logger().info(f"Errore callback Neo4j: {e}")

    def destroy_node(self):
        if self.driver:
            self.driver.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    neo4j = ServerNeo4j()
    rclpy.spin(neo4j)
    neo4j.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
