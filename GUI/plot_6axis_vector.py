import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rotate_object
import draw_drone
# Initialize serial connection (make sure the port and baud rate match your Arduino settings)
ser = serial.Serial('COM10', 115200)  # Adjust 'COM3' to your serial port
print(ser)
x_vals = []
y_vals = []
z_vals = []

w_list = []
v_list = []
u_list = []

ax_list = []
ay_list = []
az_list = []

plt.ion()  # Enable interactive mode

fig = plt.figure()
axis = plt.axes(projection ='3d')
#line, = ax.plot([], [], [], 'o')

fig_gyro = plt.figure()
ax_gyro = plt.axes(projection ='3d')


fig_acc = plt.figure()
ax_acc = plt.axes( projection ='3d')


keep_going = True
def key_capture_thread():
    global keep_going
    input()
    keep_going = False

""" ax_acc.set_xlim(-15, 15)
ax_acc.set_ylim(-15, 15)
ax_acc.set_zlim(-15, 15) """

cuboid_vertices = rotate_object.cuboid_vertices
drone_build = draw_drone.get_drone(ax_acc)
print(drone_build)

def run_graphs():
    ax_gyro.plot3D(w_list, v_list, u_list, 'green')
    ax_gyro.set_title('angles')
    ax_gyro.relim()
    ax_gyro.autoscale_view()
    ax_gyro.set_xlabel('$w$', fontsize=20, rotation=150)
    ax_gyro.set_ylabel('$v$')
    ax_gyro.set_zlabel('$u$', fontsize=30, rotation=60)
    fig_gyro.canvas.draw()
    fig_gyro.canvas.flush_events()
    
    axis.plot3D(x_vals, y_vals, z_vals, 'yellow')
    axis.set_title('Position')
    axis.relim()
    axis.autoscale_view()
    axis.set_xlabel('$x$', fontsize=20, rotation=150)
    axis.set_ylabel('$y$')
    axis.set_zlabel('$z$', fontsize=30, rotation=60)
    fig.canvas.draw()
    fig.canvas.flush_events()

counter=0
Running=True
while Running:
    
    try:
        data = ser.readline().decode().strip().split(',')

        if len(data) == 21:
            
            x, y, z = float(data[12][3:]), float(data[13][3:]), float(data[14][3:])
       #     w, v, u = float(data[6][3:])/3.14*360, float(data[7][3:])/3.14*360, float(data[8][3:])/3.14*360
           # w, v, u = float(data[6][7:]), float(data[7][6:]), float(data[8][8:])
            w, v, u = float(data[18][3:]), float(data[19][3:]), float(data[20][3:])
            ax, ay, az = float(data[0][3:]), float(data[1][3:]), float(data[2][3:])
          #  x_vals.append(x)
         #   y_vals.append(y)
          #  z_vals.append(z)
            
          #  w_list.append(w)
         #   v_list.append(v)
            print("W: " + str(w) + " v: " + str(v) + " u: " + str(u))

          #  u_list.append(u)
            
     #       ax_list.append(ax)
          #  ay_list.append(ay)
           # az_list.append(az)
          
            """             ax_gyro.plot3D(w_list, v_list, u_list, 'green')
            ax_gyro.set_title('angles')
            ax_gyro.relim()
            ax_gyro.autoscale_view()
            ax_gyro.set_xlabel('$w$', fontsize=20, rotation=150)
            ax_gyro.set_ylabel('$v$')
            ax_gyro.set_zlabel('$u$', fontsize=30, rotation=60)
            fig_gyro.canvas.draw()
            fig_gyro.canvas.flush_events() """
            
            """   axis.plot3D(x_vals, y_vals, z_vals, 'yellow')
            axis.set_title('Position')
            axis.relim()
            axis.autoscale_view()
            axis.set_xlabel('$x$', fontsize=20, rotation=150)
            axis.set_ylabel('$y$')
            axis.set_zlabel('$z$', fontsize=30, rotation=60)
            fig.canvas.draw()
            fig.canvas.flush_events() """
            plt.pause(0.01)
            counter += 1
            if (counter == 10):   
                counter=0
                         
                ax_acc.cla() 
                ax_acc = plt.gca()
                ax_acc.set_xlim([-15, 15])
                ax_acc.set_ylim([-15, 15])
                ax_acc.set_zlim([-15, 15])
        #     ax_acc.plot3D(ax, ay, az, 'yellow')

                ax_acc.set_title('acceleration')
        
                ax_acc.set_xlabel('$x$', fontsize=20, rotation=0)
                ax_acc.set_ylabel('$y$', fontsize=20)
                ax_acc.set_zlabel('$z$', fontsize=20, rotation=0)
                fig_acc.canvas.draw()
                fig_acc.canvas.flush_events()
                
                start = [0,0,0]
                ax_acc.quiver(start[0], start[1], start[2], 0, 0, az, color='r', linewidth=2)
                ax_acc.quiver(start[0], start[1], start[2], 0, ay, 0, color='g', linewidth=2)
                ax_acc.quiver(start[0], start[1], start[2], ax, 0, 0, color='y', linewidth=2)

                cube = rotate_object.rotate_cube(cuboid_vertices, [w-3.14/2,u-3.14/2,0])
                drone_arm1 = rotate_object.rotate_cube(drone_build[0], [w-3.14/2,u-3.14/2,0])
                drone_arm2 = rotate_object.rotate_cube(drone_build[1], [w-3.14/2,u-3.14/2,0])
                rotate_object.draw_cube(ax_acc, drone_arm1)
                rotate_object.draw_cube(ax_acc, drone_arm2)
                rotate_object.draw_cube(ax_acc, cube)
            


    except KeyboardInterrupt:
        print("Plotting interrupted. Closing plot.")
        plt.close(fig)  # Ensure the plot is closed cleanly
        break


ser.close()
plt.ioff()
