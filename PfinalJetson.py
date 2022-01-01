# UNIVERSIDAD AUTÓNOMA CHAPINGO
# DEPARTAMENTO DE INGENIERÍA MECÁNICA AGRÍCOLA
# INGENIERÍA MECATRÓNICA AGRÍCOLA
# DINÁMICA Y CONTROL DE ROBOTS
# PROYECTO FINAL - CONTROL ROBOT SCARA
# ALFONSO LÓPEZ HERNÁNDEZ 7° 7

# Importamos librerias
import numpy as np
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt # Importamos matplotlib como plt para graficar
from PIL import Image
from PIL import ImageTk
from tkinter import *
from tkinter import ttk
from mpl_toolkits import mplot3d # Importamos mplot3d desde mplt_toolkits subpaquete de la librería de graficación para 3D
from matplotlib import cm # Importamos cm desde matplotlib subpaquete para la graficación
from matplotlib.widgets import Slider # Subpaquete para agregar las barras de control del robot
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from adafruit_servokit import ServoKit
import time

pca = ServoKit(channels=16)

pca.servo[8].set_pulse_width_range(520, 2450)
pca.servo[10].set_pulse_width_range(520, 2450)
pca.servo[13].set_pulse_width_range(520, 2450)

rojo_bajo = np.array([110,81,71])
rojo_alto = np.array([149,255,255])

azul_bajo = np.array([3,91,139])
azul_alto = np.array([44,172,202])

verde_bajo = np.array([34,74,115])
verde_alto = np.array([64,255,255])

root = Tk()
root.title('Control Robot Scara')
root.geometry("800x600")

# Ventana principal
main_frame = Frame(root)
main_frame.pack(fill=BOTH,expand=1)
# Create Frame for X Scrollbar
sec = Frame(main_frame)
sec.pack(fill=X,side=BOTTOM)
# Create A Canvas
my_canvas = Canvas(main_frame)
my_canvas.pack(side=LEFT,fill=BOTH,expand=1)
# Add A Scrollbars to Canvas
x_scrollbar = ttk.Scrollbar(sec,orient=HORIZONTAL,command=my_canvas.xview)
x_scrollbar.pack(side=BOTTOM,fill=X)
y_scrollbar = ttk.Scrollbar(main_frame,orient=VERTICAL,command=my_canvas.yview)
y_scrollbar.pack(side=RIGHT,fill=Y)
# Configure the canvas
my_canvas.configure(xscrollcommand=x_scrollbar.set)
my_canvas.configure(yscrollcommand=y_scrollbar.set)
my_canvas.bind("<Configure>",lambda e: my_canvas.config(scrollregion= my_canvas.bbox(ALL))) 
# Create Another Frame INSIDE the Canvas
second_frame = Frame(my_canvas)
# Add that New Frame a Window In The Canvas
my_canvas.create_window((0,0),window= second_frame, anchor="nw")

# X
def matriz_traslacion_x(x): # Definimos la matriz de traslación en x
	traslacion_x = np.array([[1,0,0,x], # Primera fila de la matriz de traslación
						   [0,1,0,0], # Segunda fila de la matriz de traslación
						   [0,0,1,0], # Tercera fila de la matriz de traslación	   
						   [0,0,0,1]]) # Cuarta fila de la matriz de traslación
	return traslacion_x # Devolvemos la matriz de traslación en x

# Y
def matriz_traslacion_y(y):  # Definimos la matriz de traslación en y
	traslacion_y = np.array([[1,0,0,0], # Primera fila de la matriz de traslación
						   [0,1,0,y], # Segunda fila de la matriz de traslación
						   [0,0,1,0], # Tercera fila de la matriz de traslación	   
						   [0,0,0,1]]) # Cuarta fila de la matriz de traslación
	return traslacion_y # Devolvemos la matriz de traslación en y

# Z
def matriz_traslacion_z(z):  # Definimos la matriz de traslación en z
	traslacion_z = np.array([[1,0,0,0], # Primera fila de la matriz de traslación
						   [0,1,0,0], # Segunda fila de la matriz de traslación
						   [0,0,1,z], # Tercera fila de la matriz de traslación	   
						   [0,0,0,1]]) # Cuarta fila de la matriz de traslación
	return traslacion_z # Devolvemos la matriz de traslación en z


# X
def matriz_rotacion_x(grados_x): #Definimos la función de rotación en x
	rad = grados_x/180*np.pi # Conversión a grados
	matriz_rotacion_x = np.array([[1, 0, 0,0], #Matriz de rotación primera fila
						  [0,  np.cos(rad), -np.sin(rad),0], #Matriz de rotación segunda fila
						  [0,np.sin(rad),np.cos(rad),0], #Matriz de rotación terca fila
						  [0,0,0,1]]) # Cuarta fila de la matriz

	return matriz_rotacion_x #Devuelvo la matriz de rotación en x

# Y
def matriz_rotacion_y(grados_y): #Definimos la función de rotación en y
	rad = grados_y/180*np.pi # Conversión a grados
	matriz_rotacion_y = np.array([[np.cos(rad), 0, -np.sin(rad),0], #Matriz de rotación primera fila
						  [0,  1, 0,0], #Matriz de rotación segunda fila
						  [np.sin(rad), 0, np.cos(rad),0], #Matriz de rotación terca fila
						  [0,0,0,1]]) # Cuarta fila de la matriz

	return matriz_rotacion_y #Devuelvo la matriz de rotación en y

# Z
def matriz_rotacion_z(grados_z): #Definimos la función de rotación en z
	rad = grados_z/180*np.pi # Conversión a grados
	rotacion_z = np.array([[np.cos(rad),-np.sin(rad),0,0],  #Matriz de rotación primera fila
							[np.sin(rad),np.cos(rad),0,0], #Matriz de rotación segunda fila
								[0,0,1,0], #Matriz de rotación terca fila
								[0,0,0,1]]) # Cuarta fila de la matriz
	return rotacion_z # devolvemos la matriz de rotación en z


def cinematica_inversa(x,y,a1,a2):
	theta_2 = math.acos((x**2+y**2-a1**2-a2**2)/(2*a1*a2))
	theta_1 = math.atan2(y,x)-math.atan2((a2*math.sin(theta_2)),(a1+a2*math.cos(theta_2)))

	theta_1 = round((theta_1*180/np.pi),1)
	theta_2 = round((theta_2*180/np.pi),1)
	return theta_1,theta_2

# Configutación de la gráfica
def configuracion_grafica(): # función de configuración de la gráfica
    ax.set_title('Robot Planar Scara')
    ax.set_xlim(-17,17) # Límites en el eje x
    ax.set_ylim(0,17) # Límites en el eje y
    ax.set_zlim(-0,5) # Límites en el eje z
    
    ax.set_xlabel("x(t)") # Nombre del eje x
    ax.set_ylabel("y(t)") # Nombre del eje y
    ax.set_zlabel("z(t)") # Nombre del eje z
    ax.view_init(elev = 83, azim = -90) # # Tipo de vista de la gráfica
    
def sistema_coordenadas(a,b,c,a_f,b_f,c_f):
    x = [a,a_f] 
    y = [b,b_f]
    z = [c,c_f]

    ax.plot3D(x,[b,b],[c,c],color="red") # X
    ax.plot3D([a,a],y,[c,c],color="blue") # Y
    ax.plot3D([a,a],[b,b],z,color="green") # Z

# Sistema de coordenadas móvil para la matriz de rotación
def sistema_coordenadas_movil(matriz_rotacion): # definimos la matriz
    r_11 = matriz_rotacion[0,0] # Columna 0, Fila 0
    r_12 = matriz_rotacion[1,0] # Columna 1, Fila 0
    r_13 = matriz_rotacion[2,0] # Columna 2, Fila 0
    r_21 = matriz_rotacion[0,1] # Columna 0, Fila 1
    r_22 = matriz_rotacion[1,1] # Columna 1, Fila 1
    r_23 = matriz_rotacion[2,1] # Columna 2, Fila 1
    r_31 = matriz_rotacion[0,2] # Columna 0, Fila 2
    r_32 = matriz_rotacion[1,2] # Columna 1, Fila 2
    r_33 = matriz_rotacion[2,2] # Columna 2, Fila 2 
    dx = matriz_rotacion[0,3] # Columna 0, Fila 3
    dy = matriz_rotacion[1,3] # Columna 1, Fila 3
    dz = matriz_rotacion[2,3] # Columna 2, Fila 3

    # Sistema que va a mover
    ax.plot3D([dx,dx+r_11],[dy,dy+r_12],[dz,dz+r_13], color="m") # X 
    ax.plot3D([dx,dx+r_21],[dy,dy+r_22],[dz,dz+r_23], color="c") # Y
    ax.plot3D([dx,dx+r_31],[dy,dy+r_32],[dz,dz+r_33], color="k") # Z


def denavit_hatemberg(theta_i,d_i,a_i,alpha_i):
    MT = matriz_rotacion_z(theta_i)@matriz_traslacion_z(d_i)@matriz_traslacion_x(a_i)@matriz_rotacion_x(alpha_i)
    return MT

def robot_RR(theta_1,d1,a1,alpha_1,theta_2,d2,a2,alpha_2):
    A0 = np.eye(4) # Matriz identidad de 4x4
    A_0_1 = denavit_hatemberg(theta_1,d1,a1,alpha_1)
    A_1_2 = denavit_hatemberg(theta_2,d2,a2,alpha_2)
    A_0_2 = A_0_1 @ A_1_2   

    #print(A_0_2)   

    sistema_coordenadas_movil(A0) # sistema móvil de la base
    sistema_coordenadas_movil(A_0_1) # sistema móvil del eslabón 1
    sistema_coordenadas_movil(A_0_2) # sistema móvil del eslabón 2

    ax.plot3D([A0[0,3],A_0_1[0,3]], [A0[1,3],A_0_1[1,3]], [A0[2,3],A_0_1[2,3]], color="blue") # Eslabón 1
    ax.plot3D([A_0_1[0,3],A_0_2[0,3]], [A_0_1[1,3],A_0_2[1,3]], [A_0_1[2,3],A_0_2[2,3]], color="green") # Eslabón 2
    return A_0_2

def actualizacion_Inv(val):
    if tipo.get() == 0:  
        ax.cla() # Limpia la gráfica
        configuracion_grafica()

        x = slider_x.get()
        y = slider_y.get()
        z = slider_z.get()

        try:
            theta_1,theta_2 = cinematica_inversa(x,y,8,8.5)
            if (theta_1 > 0 and theta_1 < 180 and theta_2 > -90 and theta_2 < 90):
                Matriz_TH =  robot_RR(theta_1,0,8,0,theta_2,0,8.5,0) #theta_1, d1=0, a1=10, alpha_1=0, theta_2, d2=0, a2=7, alpha_2=0
                canvas.draw()
                plt.pause(0.001)
                slider_T_1.set(round(theta_1))
                slider_T_2.set(round(theta_2))
                slider_T_3.set(round(z))

                label_ok.config(bg= "green")
                label_ok.config(text = "")
            else:
                label_ok.config(bg="orange")
                label_ok.config(text = "Limite Angulo")

        except ValueError:
                label_ok.config(bg="red")
                label_ok.config(text = "Limite Distancia")

# Actualización de barras deslizantes para la cinematica directa
def actualizacion_Dir(val):
    if tipo.get() == 1:
        ax.cla() # Limpia la gráfica
        configuracion_grafica()
        theta_1 = slider_T_1.get()
        theta_2 = slider_T_2.get()
        Matriz_TH =  robot_RR(theta_1,0,8,0,theta_2,0,8.5,0) #theta_1, d1=0, a1=10, alpha_1=0, theta_2, d2=0, a2=7, alpha_2=0
        canvas.draw() # Actualizamos grafico
        plt.pause(0.001)
        
        sx = Matriz_TH[0,3]
        sy = Matriz_TH[1,3]
        sz = slider_z.get()

        slider_x.set(round(sx,1))
        slider_y.set(round(sy,1))
        slider_z.set(round(sz,1))

# Mover robot con coordenadas actuales
def moverRR():
    pca.servo[8].set_pulse_width_range(520, 2450)
    pca.servo[10].set_pulse_width_range(520, 2450)
    pca.servo[13].set_pulse_width_range(520, 2450)
    pca.servo[8].angle=slider_T_1.get()
    pca.servo[10].angle=slider_T_2.get()+90
    pca.servo[13].angle=slider_T_3.get()

# Inicio de la camara
def iniciar():
    global cap
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    #address = "http://192.168.1.66:8080/video"  # Direccion IP de la camara
    address = "http://192.168.1.70:81/stream"  # Direccion IP ESP32-CAM
    
    cap.open(address)  # Iniciamos captura de la camara IP
    visualizar()

# Inicio camara para detección de rojo
def iniciarR():
    global cap
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    #address = "http://192.168.1.66:8080/video"  # Direccion IP de la camara
    address = "http://192.168.1.70:81/stream"  # Direccion IP ESP32-CAM
    
    cap.open(address)  # Iniciamos captura de la camara IP
    m_rojo()

# Inicio camara para detección de verde
def iniciarV():
    global cap
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    #address = "http://192.168.1.66:8080/video"  # Direccion IP de la camara
    address = "http://192.168.1.70:81/stream"  # Direccion IP ESP32-CAM
    
    cap.open(address)  # Iniciamos captura de la camara IP
    m_verde()


# Inicio camara para detección de azul
def iniciarA():
    global cap
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    #address = "http://192.168.1.66:8080/video"  # Direccion IP de la camara
    address = "http://192.168.1.70:81/stream"  # Direccion IP ESP32-CAM
    
    cap.open(address)  # Iniciamos captura de la camara IP
    m_azul()

# Visualización de la camara 
def visualizar():
    global cap
    if cap is not None:
        ret, frame = cap.read()
        if ret == True:
            #frame = imutils.resize(frame, width=640)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(frame)
            img = ImageTk.PhotoImage(image=im)
            lblVideo.configure(image=img)
            lblVideo.image = img
            lblVideo.after(10, visualizar)
        else:
            lblVideo.image = ""
            cap.release()

# Cerrar camara
def finalizar():
    global cap
    cap.release()

# Cerrar todo
def terminar():
    root.quit()     
    root.destroy()

cap = None

# Creamos interfaz 
# Graficación en 3d
fig, ax = plt.subplots() # Creamos una figura y sus ejes
ax = plt.axes(projection = "3d")
ax.set_title('Robot Planar Scara')

canvas = FigureCanvasTkAgg(fig, master=second_frame)  # CREAR AREA DE DIBUJO DE TKINTER.
canvas.draw()
canvas.get_tk_widget().place(x=50, y=450)

x_s = IntVar()
y_s = IntVar()
z_s = IntVar()

t1_s = IntVar()
t2_s = IntVar()
t3_s = IntVar()

btnFinalizarVentana = Button(second_frame, text="Cerrar", width=20, height=2, command=terminar, bg='red')
btnFinalizarVentana.grid(column=0, row=0, padx=5, pady=20)

btnIniciarVideo = Button(second_frame, text="Iniciar Cámara", width=20, height=2, command=iniciar)
btnIniciarVideo.grid(column=1, row=0, padx=5, pady=20)

btnFinalizarVideo = Button(second_frame, text="Finalizar Cámara", width=20, height=2, command=finalizar)
btnFinalizarVideo.grid(column=2, row=0, padx=5, pady=20)

button_mover = Button(second_frame, text = "Mover", width=12, command = moverRR, bg='#3FEC4C')
button_mover.grid(column=1, row=14, padx=5, pady=20)

label_Inv = Label(second_frame,text="Cinematica Inversa")
label_Inv.grid(column=0, row=1)

slider_x = Scale(second_frame, variable = x_s, from_=-16.5, to=16.5, resolution=0.1, length=400, orient=HORIZONTAL, command=actualizacion_Inv)
slider_x.set(7)
slider_x.grid(column=1, row=11)

slider_y = Scale(second_frame, variable = y_s, from_=0, to=16.5, resolution=0.1, length=400, orient=HORIZONTAL, command=actualizacion_Inv)
slider_y.set(14)
slider_y.grid(column=1, row=12)

slider_z = Scale(second_frame, variable = z_s, from_=0, to=180, length=400, orient=HORIZONTAL, command=actualizacion_Inv)
slider_z.grid(column=1, row=13)

label_x = Label(second_frame,text="X")
label_x.grid(column=0, row=11)

label_y = Label(second_frame,text="Y")
label_y.grid(column=0, row=12)

label_z = Label(second_frame,text="Z")
label_z.grid(column=0, row=13)

label_Dir = Label(second_frame,text="Cinematica Directa")
label_Dir.grid(column=0, row=14)

slider_T_1 = Scale(second_frame, variable = t1_s, from_=0, to=180, length=400, orient=HORIZONTAL, command = actualizacion_Dir)
slider_T_1.set(90)
slider_T_1.grid(column=1, row=15)

slider_T_2 = Scale(second_frame, variable = t2_s, from_=-90, to=90, length=400, orient=HORIZONTAL, command = actualizacion_Dir)
slider_T_2.set(90)
slider_T_2.grid(column=1, row=16)

slider_T_3 = Scale(second_frame, variable = t3_s, from_=0, to=180, length=400, orient=HORIZONTAL, command = actualizacion_Dir)
slider_T_3.set(90)
slider_T_3.grid(column=1, row=17)

label_T_1 = Label(second_frame,text="Theta_1")
label_T_1.grid(column=0, row=15)

label_T_2 = Label(second_frame,text="Theta_2")
label_T_2.grid(column=0, row=16)

label_T_3 = Label(second_frame,text="Theta_3")
label_T_3.grid(column=0, row=17)

label_ok = Label(second_frame,text=" ", height=2, width=20,bg="green")
label_ok.grid(column=1, row=1, padx=5, pady=20)#.place(x=600, y=200)

tipo = IntVar()
R1 = Radiobutton(second_frame, text="Inversa Activa", variable=tipo, value=0, height=3, width=10)
R2 = Radiobutton(second_frame, text="Directa Activa", variable=tipo, value=1, height=3, width=10)
R1.place(x=580, y=80)
R2.place(x=580, y=120)

lblVideo = Label(second_frame)
#lblVideo.grid(column=1, row=20)
lblVideo.place(x=830, y=300)



# Funciones para guardar y repetir puntos
lista_Theta1 = []
lista_Theta2 = []
lista_Theta3 = []

def guardar():
    
    global lista_Theta1
    global lista_Theta2
    global lista_Theta3

    lista_Theta1.append(slider_T_1.get())
    lista_Theta2.append(slider_T_2.get()+90)
    lista_Theta3.append(slider_T_3.get())

def vaciarPuntos():
    global lista_Theta1
    global lista_Theta2
    global lista_Theta3
    lista_Theta1.clear()
    lista_Theta2.clear()
    lista_Theta3.clear()

def recorrer():
    global lista_Theta1
    global lista_Theta2
    global lista_Theta3
    pca.servo[8].set_pulse_width_range(520, 2450)
    pca.servo[10].set_pulse_width_range(520, 2450)
    pca.servo[13].set_pulse_width_range(520, 2450)

    for paso in range(len(lista_Theta1)):
        pca.servo[8].angle=lista_Theta1[paso]
        pca.servo[10].angle=lista_Theta2[paso]
        pca.servo[13].angle=lista_Theta3[paso]
        time.sleep(1)

# Detección de color rojo
def m_rojo():
    global cap
    global x_f
    global y_f

    if cap is not None:
        ret, frame = cap.read()
        if ret == True:
            rgb = frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, rojo_bajo, rojo_alto)
            contornos, __ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(rgb,contornos,-1,(255,0,0),3)

            for c in contornos:
                area = cv2.contourArea(c)
                if area > 250: # área del objeto
                    # Encontramos los momentos del objeto
                    M = cv2.moments(c)
                    if M["m00"] == 0:
                        M["m00"] = 1 # Igualamos a 1 ya que si dividimos entre 0 sería una indefinición
                    # Obtenemos coordenadas del objeto
                    x = int(M["m10"]/M["m00"])
                    y = int(M["m01"]/M["m00"])
                    
                    num_px_cm = 21
                    x_2 = x/num_px_cm
                    y_2 = y/num_px_cm

                    # Coordenadas corregidas en cm
                    x_f = 18.5-x_2
                    y_f = y_2-2.5
                    print(x_f,y_f)

            im = Image.fromarray(rgb)
            img = ImageTk.PhotoImage(image=im)
            lblVideo.configure(image=img)
            lblVideo.image = img
            lblVideo.after(10, m_rojo)
        else:
            lblVideo.image = ""
            cap.release()


# Detección de color verde
def m_verde():
    global cap
    global x_f
    global y_f

    if cap is not None:
        ret, frame = cap.read()
        if ret == True:
            rgb = frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, verde_bajo, verde_alto)
            contornos, __ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(rgb,contornos,-1,(255,0,0),3)

            for c in contornos:
                area = cv2.contourArea(c)
                if area > 250: # área del objeto
                    # Encontramos los momentos del objeto
                    M = cv2.moments(c)
                    if M["m00"] == 0:
                        M["m00"] = 1 # Igualamos a 1 ya que si dividimos entre 0 sería una indefinición

                    # Obtenemos coordenadas del objeto
                    x = int(M["m10"]/M["m00"])
                    y = int(M["m01"]/M["m00"])
                    
                    num_px_cm = 21
                    x_2 = x/num_px_cm
                    y_2 = y/num_px_cm

                    # Coordenadas corregidas en cm
                    x_f = 18.5-x_2
                    y_f = y_2-2.5
                    print(x_f,y_f)


            im = Image.fromarray(rgb)
            img = ImageTk.PhotoImage(image=im)
            lblVideo.configure(image=img)
            lblVideo.image = img
            lblVideo.after(10, m_verde)
        else:
            lblVideo.image = ""
            cap.release()


# Detección de color azul
def m_azul():
    global cap
    global x_f
    global y_f

    if cap is not None:
        ret, frame = cap.read()
        if ret == True:
            rgb = frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, azul_bajo, azul_alto)
            contornos, __ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(rgb,contornos,-1,(255,0,0),3)

            for c in contornos:
                area = cv2.contourArea(c)
                if area > 250: # área del objeto
                    # Encontramos los momentos del objeto
                    M = cv2.moments(c)
                    if M["m00"] == 0:
                        M["m00"] = 1 # Igualamos a 1 ya que si dividimos entre 0 sería una indefinición
                    # Obtenemos coordenadas del objeto
                    x = int(M["m10"]/M["m00"])
                    y = int(M["m01"]/M["m00"])
                    
                    num_px_cm = 21
                    x_2 = x/num_px_cm
                    y_2 = y/num_px_cm

                    # Coordenadas corregidas en cm
                    x_f = 18.5-x_2
                    y_f = y_2-2.5

                    print(x_f,y_f)


            im = Image.fromarray(rgb)
            img = ImageTk.PhotoImage(image=im)
            lblVideo.configure(image=img)
            lblVideo.image = img
            lblVideo.after(10, m_azul)
        else:
            lblVideo.image = ""
            cap.release()


# Mover ficha de color detectada
def ficha():
    global x_f
    global y_f
    theta_1,theta_2 = cinematica_inversa(x_f,y_f,8,8.5)
    pca.servo[8].set_pulse_width_range(520, 2450)
    pca.servo[10].set_pulse_width_range(520, 2450)
    pca.servo[13].set_pulse_width_range(520, 2450)

    pca.servo[8].angle=theta_1
    pca.servo[10].angle=theta_2 + 90
    time.sleep(0.5)
    pca.servo[13].angle=0
    time.sleep(0.5)
    pca.servo[13].angle=180
    time.sleep(1)
    pca.servo[8].angle=75
    pca.servo[10].angle=70 + 90
    time.sleep(0.5)
    pca.servo[13].angle=0
    time.sleep(0.5)
    pca.servo[13].angle=180
    time.sleep(0.1)
  
# Movemos a la posición inicial el robot
def home():
    tipo.set(1)
    if tipo.get() == 1:
        ax.cla() # Limpia la gráfica
        configuracion_grafica()
        theta_1 = 44
        theta_2 = 37
        Matriz_TH =  robot_RR(theta_1,0,8,0,theta_2,0,8.5,0) #theta_1, d1=0, a1=10, alpha_1=0, theta_2, d2=0, a2=7, alpha_2=0
        canvas.draw()
        plt.pause(0.001)
        #print(Matriz_TH)
        sx = Matriz_TH[0,3]
        sy = Matriz_TH[1,3]
        sz = slider_z.get()

        slider_x.set(round(sx,1))
        slider_y.set(round(sy,1))
        slider_z.set(90)

        pca.servo[8].set_pulse_width_range(520, 2450)
        pca.servo[10].set_pulse_width_range(520, 2450)
        pca.servo[13].set_pulse_width_range(520, 2450)

        pca.servo[8].angle=theta_1
        pca.servo[10].angle=theta_2 + 90
        pca.servo[13].angle=90

        tipo.set(0)
        slider_T_1.set(theta_1)
        slider_T_2.set(theta_2)
        slider_T_3.set(90)
        tipo.set(1)

# Seguimiento de trayyectoria
def trayecto():
    # y = 0*x + b
    # y = 13
    y = 13
    for x in np.arange(-10,10,.5):
        ax.cla() # Limpia la gráfica
        configuracion_grafica()
        theta1,theta2 = cinematica_inversa(x,y,8,8.5)
        Matriz_TH = robot_RR(theta1,0,8,0,theta2,0,8.5,0)

        pca.servo[8].angle = theta1
        pca.servo[10].angle = theta2+90
        pca.servo[13].angle = 180

        canvas.draw()
        time.sleep(0.05)


button_guardar = Button(second_frame, text = "Guardar Posición", width=12, command = guardar)
button_guardar.place(x=600, y=180)
button_vaciar = Button(second_frame, text = "Vaciar Datos Guardados", width=18, command = vaciarPuntos)
button_vaciar.place(x=580, y=210)
button_recorrer = Button(second_frame, text = "Recorrer Datos Guardados", width=20, command = recorrer)
button_recorrer.place(x=580, y=240)

button_home = Button(second_frame, text = "Home", width=12, command = home)
button_home.place(x=450, y=90)

button_linea = Button(second_frame, text = "Trayectoria", width=12, command = trayecto)
button_linea.place(x=450, y=260)

button_rojo = Button(second_frame, text = "Buscar Rojo", width=12, command = iniciarR)
button_rojo.place(x=600, y=300)
button_verde = Button(second_frame, text = "Buscar verde", width=12, command = iniciarV)
button_verde.place(x=600, y=330)
button_azul = Button(second_frame, text = "Buscar azul", width=12, command = iniciarA)
button_azul.place(x=600, y=360)
button_ficha = Button(second_frame, text = "Mover Ficha", width=12, command = ficha)
button_ficha.place(x=600, y=390)

# Aumentar tamaño de la ventana con labels
# Horizontal
for thing in range(40):
    Label(second_frame ,text="").grid(row=1,column=thing+3,pady=10,padx=10)
#Vertical
for thing in range(20):
    Label(second_frame ,text="").grid(row=thing+20,column=6,pady=10,padx=10)

# Cerrar grafico de matplotlib
plt.close(1)
# Poner ventana al frente
root.wm_attributes("-topmost", True)
# Ejecutar interfaz
root.mainloop()