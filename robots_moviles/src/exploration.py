#!/usr/bin/env python

import rospy #importar libreria de ROS
import actionlib #importar libreria de actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #importar mensajes de move_base
from nav_msgs.msg import OccupancyGrid #importar mensajes de mapa
from random import randint #importar libreria para generar numeros aleatorios
from sensor_msgs.msg import PointCloud #importar mensajes de nube de puntos
from geometry_msgs.msg import Point32 #importar mensajes de puntos
from geometry_msgs.msg import PoseStamped #importar mensajes de pose
import random #importar libreria para generar numeros aleatorios
import actionlib #importar libreria de actionlib
import actionlib_msgs.msg #importar mensajes de actionlib

map_data = None #inicializar el mapa como nulo

#FUNCION CALLBACK DE SUSCRIPCION AL TOPIC DEL MAPA
def map_callback(map_msg): 
    global map_data #definir el mapa como global, para poder accederlo en otras funciones
    print('Mapa recibido')
    map_data = map_msg #guardar el mapa en el mensaje 
    
#ALGORITMO DE EXPLORACIÓN (ELEGIR DESTINO Y NAVEGAR HASTA ÉL)
def select_and_publish_goal(map_data): 

    if map_data is not None: #si el mapa no es nulo

        print('Eligiendo destino...')
        width = map_data.info.width #guardar el ancho del mapa
        height = map_data.info.height #guardar el alto del mapa

        #ALGORITMO PARA ENCONTRAR TODAS LAS CELDAS FRONTERA

        frontiers = [] # Inicializar la lista de fronteras
        for y in range(height): # Iterar sobre el mapa para encontrar las fronteras
            for x in range(width):
                index = y * width + x # Calcular el indice de la celda
                if map_data.data[index] == -1: # Si la celda es desconocida
                    for dy in [-1, 0, 1]: # Iterar sobre las celdas vecinas
                        for dx in [-1, 0, 1]: 
                            if dx == 0 and dy == 0: # Si la celda es la misma, saltar a la siguiente
                                continue
                            nx, ny = x + dx, y + dy # Calcular las coordenadas de la celda vecina
                            if nx >= 0 and nx < width and ny >= 0 and ny < height: # Si la celda vecina esta dentro del mapa
                                nindex = ny * width + nx # Calcular el indice de la celda vecina
                                if map_data.data[nindex] == 0: # Si la celda vecina es libre
                                    frontiers.append((x, y)) # Agregar la celda desconocida a la lista de fronteras
                                    break
                        else: # Si la celda vecina es desconocida, continuar
                            continue
                        break # Si la celda vecina es libre, salir del bucle, se ha encontrado una frontera

                    
        # Crear un mensaje de tipo PointCloud para publicar las fronteras
        frontier_msg = PointCloud()
        frontier_msg.header.frame_id = "map"
        frontier_msg.header.stamp = rospy.Time.now()

        # #Añadir las fronteras al mensaje
        for x, y in frontiers:
            # Convertir las coordenadas de la celda a coordenadas del mundo
            world_x = x * map_data.info.resolution + map_data.info.origin.position.x
            world_y = y * map_data.info.resolution + map_data.info.origin.position.y
            frontier_msg.points.append(Point32(world_x, world_y, 0))

        # Publicar las fronteras para su visualización en rviz
        frontier_pub.publish(frontier_msg)
        #--------------------------------------------------------------------------------

        #ALGORITMO PARA AGRUPAR LAS CELDAS FRONTERA EN LINEAS FRONTERIZAS
        
        groups = [] # Inicializar la lista de grupos
        visited = set() # Inicializar el conjunto de celdas visitadas
        for x, y in frontiers: # Iterar sobre las celdas de las fronteras
            if (x, y) not in visited: # Si la celda no ha sido visitada
                group = [] # Inicializar el grupo
                stack = [(x, y)] # Inicializar la pila con la celda actual
                while stack: # Mientras la pila no este vacia
                    x, y = stack.pop() # Sacar una celda de la pila
                    if (x, y) not in visited: # Si la celda no ha sido visitada
                        visited.add((x, y)) # Marcar la celda como visitada
                        group.append((x, y)) # Agregar la celda al grupo
                        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]: # Iterar sobre las celdas vecinas
                            nx, ny = x + dx, y + dy # Calcular las coordenadas de la celda vecina
                            if (nx, ny) in frontiers: # Si la celda vecina es una frontera
                                stack.append((nx, ny)) # Agregar la celda vecina a la pila
                groups.append(group) # Agregar el grupo a la lista de grupos

        #--------------------------------------------------------------------------------

        #ALGORITMO PARA ELEGIR UNA LINEA FRONTERIZA Y EL DESTINO DENTRO DE ELLA
                
        # Ordenar las lineas fronterizas por tamaño
        groups.sort(key=len, reverse=True)
        # Elegir la linea mas grande o uno de los dos mas grandes, para evitar que el robot se quede atascado
        selected_group = random.choice(groups[:2])
        # Escoger como destino, el centroide de la linea fronteriza seleccionada
        goal_x, goal_y = sum(x for x, y in selected_group) / len(selected_group), sum(y for x, y in selected_group) / len(selected_group)

        # Crear un mensaje de tipo PoseStamped para publicar el destino y verlo en rviz
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position.x = goal_x * map_data.info.resolution + map_data.info.origin.position.x
        goal_msg.pose.position.y = goal_y * map_data.info.resolution + map_data.info.origin.position.y
        goal_msg.pose.orientation.w = 1.0

        # Publicar el destino para su visualización en rviz
        goal_pub.publish(goal_msg)

        #--------------------------------------------------------------------------------

        #NAVEGACIÓN HASTA EL DESTINO

        print('Destino elegido. Navegando hasta el punto...') 

        goal_client = actionlib.SimpleActionClient('move_base',MoveBaseAction) #crear un cliente para enviar el destino
        goal_client.wait_for_server() #esperar a que el servidor esté listo
        
        goal = MoveBaseGoal() #crear un mensaje de tipo MoveBaseGoal
        goal.target_pose.header.frame_id = "map" #establecer el frame del destino
        goal.target_pose.header.stamp = rospy.Time.now() #establecer el tiempo del destino
        goal.target_pose.pose.position.x = goal_x * map_data.info.resolution + map_data.info.origin.position.x #establecer la coordenada x del destino
        goal.target_pose.pose.position.y = goal_y * map_data.info.resolution + map_data.info.origin.position.y #establecer la coordenada y del destino
        goal.target_pose.pose.orientation.w = 1.0 #establecer la orientacion del destino (cuaternion constante)
        goal_client.send_goal(goal) #enviar el destino
        finished_within_time = goal_client.wait_for_result(rospy.Duration(20)) #Esperar a que el robot llegue al destino 20s
        
        if not finished_within_time: #si no ha llegado al destino
            goal_client.cancel_goal()  #Cancelar el objetivo
            print('No se puede llegar a este objetivo, cambiamos a otro.')
        else: #si ha llegado al destino
                print('Punto alcanzado!')
                

#ALGORITMO PARA EL CRITERIO DE PARADA DE LA EXPLORACIÓN         
previous_known_cells = 1 # Inicializar el numero  previo de celdas conocidas como =! 0, para no dividir entre 0 en la primera iteracion 

def calculate_known_percentage(map_data): #FUNCION PARA CALCULAR EL PORCENTAJE DE MAPA CONOCIDO
    global previous_known_cells # Acceder a la variable global

    known_cells = sum(1 for cell in map_data.data if cell != -1) # Calcular el numero de celdas conocidas totales
    print(f'Celdas conocidas ahora: {known_cells}')

    increment = known_cells - previous_known_cells # Calcular el incremento de celdas conocidas en esta iteración
    print(f'Incremento de celdas conocidas en esta iteración: {increment}')

    if previous_known_cells != 0:  # Evitar dividir entre 0
        increment_percentage = (increment / previous_known_cells) * 100 # Calcular el porcentaje de incremento
    else:
        increment_percentage = 0 # Si no hay incremento, el porcentaje es 0

    previous_known_cells = known_cells # Actualizar el numero previo de celdas conocidas
    return increment_percentage # Devolver el porcentaje de incremento


#FUNCION PRINCIPAL
if __name__ == '__main__':
    try:
        rospy.init_node('exploration', anonymous=True) #Iniciar el nodo
        map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback) #Suscribirse al topic del mapa
        frontier_pub = rospy.Publisher('frontiers', PointCloud, queue_size=10) #Publicar las fronteras
        goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10) #Publicar el destino
        rate = rospy.Rate(1)

        while not rospy.is_shutdown(): #Bucle principal
            
            if map_data is not None: #Si el mapa no es nulo

                select_and_publish_goal(map_data) #Ejecutar el algoritmo de exploración

                #CRITERIO DE PARADA DE LA EXPLORACIÓN
                known_percentage = calculate_known_percentage(map_data) #Calcular el porcentaje de incremento de mapa conocido
                print(f'Porcentaje de incremento de mapa conocido en esta iteración: +{known_percentage}%') 

                if known_percentage==0: #Si el porcentaje de incremento de mapa conocido es 0 (ya no está aprendiendo nada nuevo)
                    print('Exploración completada. Fin de ejecución.')
                    break
            rate.sleep() #esperar

    except rospy.ROSInterruptException: #excepcion de ROS
        rospy.loginfo("Excepción de ROS. Fin de ejecución")
