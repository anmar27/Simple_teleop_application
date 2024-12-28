
import socket
from robodk.robolink import *  # RoboDK's API
from robodk.robomath import *  # Robot toolbox
import time
import threading
import json

def server_program(host='10.5.20.89', port=5000):
    # Start the RoboDK API:
    RDK = Robolink()

    # Get the robot (first robot found):
    robot = RDK.Item('', ITEM_TYPE_ROBOT)

    # Get the initial robot pose (current position):
    target_pose = robot.Pose()
    xyz_ref = target_pose.Pos()

    # Define wall boundaries (in mm)
    X_MIN, X_MAX = 155, 455
    Y_MIN, Y_MAX = -231, 20
    Z_MIN, Z_MAX = 400, 560
    
    print(f"Server listening on {host}:{port}")
    
    # Set up the socket server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((host, port))  # Bind the socket to the host and port
        server_socket.listen(1)  # Start listening for incoming connections
        conn, addr = server_socket.accept()  # Accept a connection
        with conn:
            print(f"Connection from {addr}")
            #Robolink.ShowMessage("This is a test")
            
            # Start a loop to receive data
            while True:
                data = conn.recv(1024)  # Receive data from the client
                if not data:
                    break  # Exit if no data is received
                print(f"Received from client: {data.decode()}")  # Print received data
                
                if (data.decode() == "0.000,0.000,0.000"):
                    print("Waiting for different state")
                    
                else:
                    
                    try:
                        # Assuming the data is a string with coordinates: "x,y,z"
                        position_data = data.decode().strip()  # Remove any leading/trailing whitespace
                        x_str, y_str, z_str = position_data.split(',')  # Split by commas
                        x = float(x_str)
                        y = float(y_str)
                        z = float(z_str)

                        #Create constant value 
                        K = 1000
                        x = K * x + 305.248
                        y = K * y - 131.058
                        z = K * z + 480.894
                        
                        print("Values with constant: ", x , y , z )

                        if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX and Z_MIN <= z <= Z_MAX):
                            print(f"Error: Position ({x}, {y}, {z}) is outside the safe boundaries.")
                            response = "Error: Position outside safe boundaries."
                            conn.send(response.encode())  # Notify the client
                            continue  # Skip further processing
                        # Create a new pose based on the received position
                        #orientation = target_pose.Orient() #Store current orientation
                        target_pose.setPos([x, y, z])
                        
                        #Check for feasibility with the IK solver
                        joint_solution = robot.SolveIK(target_pose)
                        if joint_solution is None:
                            print("Error: No valid joint solution found for the target pose.")
                            response = "Error: Target pose is not reachable."
                            conn.send(response.encode())  # Send an error response
                            continue

                        #robot.setSpeed(100)  # Set linear speed (mm/s)
                        robot.setAcceleration(100)  # Set linear acceleration (mm/s^2)

                        
                    # If a valid solution is found, move the robot using MoveJ
                        try:
                            robot.MoveJ(target_pose)  # Move using joint motion
                            print("Robot moved to the target position.")
                            response = "Message received and position updated."
                            conn.send(response.encode())  # Send a response back to the client
                        except Exception as e:
                            print(f"Error during robot movement: {e}")
                            response = "Error: Unable to execute the movement."
                            conn.send(response.encode())
                            
                    except ValueError:
                        print("Error: Invalid position data format.")
                        response = "Error: Invalid position data format"
                        conn.send(response.encode())

if __name__ == '__main__':
    server_program()



