#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sqlite3
import json
from wormhole_nav_msg.srv import PoseLoader

class TableCreatorNode(Node):
    def __init__(self):
        super().__init__("db_pose_fetch")

        self.conn = sqlite3.connect('json_table.db')
        self.cursor = self.conn.cursor()

        self.create_table()
        self.insert_json_data()

        self.srv = self.create_service(PoseLoader, '/wormhole/load_pose', self.fetch_json_callback)
        self.get_logger().info("Service is ready - db_pose_fetch")

    def create_table(self):
        try:
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS json_data (
                    data TEXT
                )
            ''')
            self.conn.commit()
            self.get_logger().info("Table created successfully - db_pose_fetch")
        except sqlite3.Error as e:
            self.get_logger().error(f'Error creating table: {e}')

    def insert_json_data(self):
        try:
            # Clear existing data to ensure exactly 4 rows
            self.cursor.execute('DELETE FROM json_data')
            # Sample JSON data for four rows
            json_rows = [
                {"id": 1, "name": "Item 1", "value": 100},
                {"id": 2, "name": "Item 2", "value": 200},
                {"id": 3, "name": "Item 3", "value": 300},
                {"id": 4, "name": "Item 4", "value": 400}
            ]

            # Insert JSON data
            for json_obj in json_rows:
                json_str = json.dumps(json_obj)
                self.cursor.execute('INSERT INTO json_data (data) VALUES (?)', (json_str,))
            
            self.conn.commit()
            self.get_logger().info('Inserted 4 rows of JSON data')

        except sqlite3.Error as e:
            self.get_logger().error(f'Error inserting data: {e}')

    def fetch_json_callback(self, request, response):
        row_number = request.row_number
        try:
            # Fetch the JSON data for the requested row
            self.cursor.execute('SELECT data FROM json_data LIMIT 1 OFFSET ?', (row_number - 1,))
            result = self.cursor.fetchone()
            if result:
                response.json_data = result[0]
                self.get_logger().info(f'Fetched row {row_number}: {result[0]}')
            else:
                response.json_data = f"Error: Row {row_number} does not exist"
                self.get_logger().error(response.json_data)
        except sqlite3.Error as e:
            response.json_data = f"Error: Database query failed - {e}"
            self.get_logger().error(response.json_data)
        return response

    def destroy_node(self):
        # Close database connection when node is destroyed
        self.conn.close()
        self.get_logger().info('Database connection closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TableCreatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()