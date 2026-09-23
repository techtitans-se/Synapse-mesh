import http.server
import socketserver
import subprocess
import os

PORT = 8000
DASHBOARD_DIR = os.path.dirname(os.path.abspath(__file__))

class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        '.css': 'text/css',
        '.js': 'application/javascript',
        '.html': 'text/html',
        '.json': 'application/json',
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DASHBOARD_DIR, **kwargs)

    def translate_path(self, path):
        # Normalize and support /src/amr_web_dashboard alias for both GET and HEAD
        if path.startswith('/src/amr_web_dashboard'):
            path = path[len('/src/amr_web_dashboard'):]
            if not path or path == '/':
                path = '/index.html'
        return super().translate_path(path)

    def end_headers(self):
        # Force browser to always fetch fresh files — never cache
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_POST(self):
        if self.path in ('/restart_sim', '/src/amr_web_dashboard/restart_sim'):
            print("Restarting Simulation and Executor...")
            
            # Kill existing simulation processes
            os.system("killall -9 gzserver gzclient rviz2 spawn_entity.py map_server robot_state_publisher || true")
            os.system("pkill -9 -f amr_task_executor.py || true")
            
            # Start Simulation
            subprocess.Popen(
                ['bash', '-c', 'export DISPLAY=:0 && /home/akil/Desktop/synapse_mesh/run_simulation.sh'],
                cwd='/home/akil/Desktop/synapse_mesh',
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            # Start AMR Task Executors for 3 AMRs
            for amr_id in [1, 2, 3]:
                robot_name = f'synapse_amr_{amr_id}'
                
                log_file = open(f'/tmp/{robot_name}_executor.log', 'w')
                subprocess.Popen(
                    ['bash', '-c', f'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run synapse_amr_warehouse amr_task_executor.py --ros-args -p robot_name:={robot_name}'],
                    cwd='/home/akil/Desktop/synapse_mesh',
                    stdout=log_file,
                    stderr=log_file
                )
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status": "restarted"}')
            return
            
        elif self.path in ('/restart_sim_vos', '/src/amr_web_dashboard/restart_sim_vos'):
            print("Restarting Simulation and Executor in VOS Mode...")
            
            # Kill existing simulation processes
            os.system("killall -9 gzserver gzclient rviz2 spawn_entity.py map_server robot_state_publisher || true")
            os.system("pkill -9 -f amr_task_executor.py || true")
            
            # Start Simulation
            subprocess.Popen(
                ['bash', '-c', 'export DISPLAY=:0 && /home/akil/Desktop/synapse_mesh/run_simulation.sh'],
                cwd='/home/akil/Desktop/synapse_mesh',
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            # Start AMR Task Executors for 3 AMRs in VOS Mode
            for amr_id in [1, 2, 3]:
                robot_name = f'synapse_amr_{amr_id}'
                
                log_file = open(f'/tmp/{robot_name}_executor_vos.log', 'w')
                subprocess.Popen(
                    ['bash', '-c', f'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run synapse_amr_warehouse amr_task_executor.py --ros-args -p robot_name:={robot_name} -p enable_vos:=true -p enable_st_lease:=false'],
                    cwd='/home/akil/Desktop/synapse_mesh',
                    stdout=log_file,
                    stderr=log_file
                )
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status": "restarted_vos"}')
            return
            
        elif self.path in ('/stress_test', '/src/amr_web_dashboard/stress_test'):
            print("Triggering ST-Lease Stress Test...")
            subprocess.Popen(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash && source install/setup.bash && python3 src/synapse_amr_warehouse/scripts/stress_test_st_lease.py'],
                cwd='/home/akil/Desktop/synapse_mesh',
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status": "stress_test_started"}')
            return
            
        elif self.path in ('/vos_stress_test', '/src/amr_web_dashboard/vos_stress_test'):
            print("Triggering VOS Stress Test...")
            subprocess.Popen(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash && source install/setup.bash && python3 src/synapse_amr_warehouse/scripts/stress_test_vos.py'],
                cwd='/home/akil/Desktop/synapse_mesh',
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status": "vos_stress_test_started"}')
            return
        else:
            self.send_response(404)
            self.end_headers()
            
    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()
            
    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header("Access-Control-Allow-Headers", "X-Requested-With")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

socketserver.TCPServer.allow_reuse_address = True
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"Serving {DASHBOARD_DIR} at port {PORT} with Restart API enabled...")
    httpd.serve_forever()
