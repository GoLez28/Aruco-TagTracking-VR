Primero, debo decir que mi programa no esta completo, si quieres una solucion mas refinada, moderna y facil de usar utiliza este proyecto https://github.com/ju1ce/April-Tag-VR-FullBody-Tracker,  
pero solo utiliza una camara,  
si vas a querer utilizar mi programa, vas a tener que descargar ese proyecto de todas formas, por tema de calibrar la camara

Empezando, debes instalar un driver para steamvr, que permitira que el tracker sea detectado por steamvr,  
debes instalar VMT https://github.com/gpsnmeajp/VirtualMotionTracker/releases/tag/v0.15 , el instalador es bastante facil de usar,  
una vez instalado no se necesita hacer nada mas (eso creo).
Despues, vas a tener que descargar mi programa https://github.com/GoLez28/TagTracking/releases/tag/v0.4, cuando lo abras por primera vez, se generara el archivo "arucoboard.png" al lado del .exe,  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/arucoboard.png" width="200">   
esto es lo que vas a tener que imprimir, pero solo lo que necesites, por ejemplo si vas a usar 4 para el pie derecho, 4 para el izquierdo y 6 para la cintura,  
necesitaras 14 en total, lo demas va a ser innecesario.

asegurate de que todos los tags sean del mismo tamaño, cuando ya los tengas impresos, midelos y coloca los milimetros del cuadrado negro en config.txt, trackerSize=x  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/tag%20size.jpg" width="400">  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/set tag size.png" width="200">  

pegas los tags impresos en un pedazo de carton o cualquier cosa que sirva de tal forma, 
esta es la forma que tengo puesto los trackers  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/my%20setup.jpg" width="400">  

pero tambien puedes colocarlos asi, o realmente de cualquier forma, entre mas grande, mejor es detectado, pero mas incomodo  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/simple%20tracker.jpg" width="200">  
pero te tienes que asegurar de que el segundo tag (el que esta a 1.57 radianes), sea el que este mirando hacia el frente,  
puedes modificar la posicion y cantidad de los tags, en "trackers.txt", que ya deberia estar configurado, pero el de la citura posiblemente lo tengas que editar,  
el archivo se muestra en este formato
```
NOMBRE
id angulo posX posY posZ
parametros
```
ej:
```
rightfoot
0 0.0 0.0 0.0 -0.055
1 1.5707963268 0.0 0.0 -0.055
2 3.1415926536 0.0 0.0 -0.055
3 4.7123889804 0.0 0.0 -0.055
filterSmoothRot=10
filterSmoothPos=8
```
este ejemplo es de un cuadrado perfecto para el pie derecho  
si te fijas, en posZ esta puesto -0.055 como valor, esa es la profundidad que debe tener el tag para que sea el centro del pie, cintura  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/leg%20center.jpg" width="200">  
como se muestra aqui, un profundidad de 55mm (0.055m)

Para las camaras yo estoy utilizando celulares por la app iVCam (si quieres saber como configurarlo de la mejor forma, me preguntas),  
pero primero hay que calibrar las camaras, para eso necesitamos el otro proyecto 'Juices VR Marker Tracking', por que ahi tiene una opcion para calibrar camaras,  
al abrir esa aplicacion, te vas a la pestaña Params, y en 'ID of camera' colocas 0, abajo le das Save,  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/juices%20camera%20config.png" width="200">  
te ve vas a la pestaña de Camera, le das Start, y Preview, si en la camara no se ve nada, tienes que cambiar de camara, le das Stop y te vas a Params y pones otro numero en ID of camera y repites,  
cuando el preview muestre la camara que quieres calibrar, en la pestaña de Camera, le das a Calibrate camera,  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/calib camera gui.png" width="200">  
en la carpeta dela app de calibracion, deberia haber una carpeta llamada 'images-to-print' con 'charuco_board.jpg',  
se supone que deberias imprimir eso tambien pero simplemente lo pones en pantalla completa y funciona igual,  
la camara lo pones de manera que se vea la imagen completa, y vas moviendo la camara en varias posiciones, hasta que encuentra que sea suficiente,  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/camera%20calib.jpg" width="600">  

Una vez hecho eso en la carpeta de la app de calibracion, deberia haber un archivo llamado 'calib.yaml' en la carpeta 'config',  
copias los valores del archivo 'config.yaml' a 'cameraParameters1.xml' que esta en la carpeta de la aplicacion que vamos a usar,  
hay que copiar de camMat a cameraMatrix, y de distCoeffs a dist_coeffs, fijate que xml no usa comas si no que espacios,  
(si la camara cambia en tema de resolucion, o otra cosa, se debe hacer la calibracion de nuevo)  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/calib vals.png" width="600">  

ahora toca configurar las camaras, en config.txt tienes que asegurarte de que cameraXFile sea el nombre del xml de calibracion (un xml para cada camara), seguido de la resolucion y ID (Index),  
lo mismo para todas las camaras que vas a usar definido por totalCameras (entre mas camaras, resolucion, fps, mas CPU va a usar),  
en mi parecer 960p en 4:3 es lo mejor para una camara, 600p es pasable, pero 480p es simplemente muy impreciso  

Ahora que todo esta configurado, es momento de abrir SteamVR, para que la aplicacion funcione correctamente, o si no saldra error de OVR no detectado  
(que funcionaria igual, pero la compensacion de latencia del visor no se aplicaria)  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/ovr error.png" width="400">  
si todo funciona correctamente, steamvr deberia detectar los nuevos tracker al iniciar la aplicacion  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/vmt ok.png" width="400">  
si no se muestran los trackers VMT es porque los driver se instalaron mal, los puerto no estan siendo detectados, o steamvr los bloqueo que en ese caso se arregla en configuraciones  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/vmt unlock.png" width="400">  

cuando tengas las camaras mirando hacia el espacio que vas a usar, preferiblemente uno en cada esquina, uno bajo y otro alto (equinas opuestas puede que no calibre bien),  
y los tracker puestos en tus piernas y cintura (si gustas puedes añadirte mas), es momento de empezar calibracion.  
(para que sea mas comodo, ocupa el teclado en pantalla de windows, viendo el escritorio en steamVR. o el teclado de OVRToolkit o XSOverlay funcionan, el de SteamVR no) Presiona '1' para empezar el modo calibracion, debes situarte en una parte del espacio de juego, y asegurarse de que almenos un tag sea visible para ambas camaras (puedes visualizar la camara presionando '9', pero no lo pretes de nuevo que crasheara jeje)  
esto indicara un 'Tracker OK',  mantente quieto en esa sona y tendras que presionar '2' para registarlo,  (puedes presionar '3' para que sea automatico)  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/tracker info.png" width="200">  

hace esto para rellenar el mayor espacio posible en tu zona,  
cuando ya creas que son suficientes tags registrados, asegurate de que el tag numero 0 sea visible para las dos camaras (idealmente puesta en el suelo), vuelve a presionar '1' para empezar la calibracion, y espera hasta que diga 'Ended Searching for matrices' y 'Saving...',  
<img src="https://raw.githubusercontent.com/GoLez28/TagTracking/master/tutorial/save cams.png" width="600">  
eso calibrara las camaras, pero no la posicion de los trackers, si ves que los trackers se agitan / tiritan mucho, es posible que la calibracion no fue muy buena y tenga que hacerse de nuevo,  
o de que la camara entrega un resultado muy ruidoso.  

Una vez con las camara calibradas, parate derecho y presiona '5' para que haga una estimacion de donde deberian estar los trackers, si quedas insatifecho con los resultados (osea siempre xD), puedes usar el teclado para refir la posiciones, 'q','w','a','s','z','x' para cambiar la posicion y 'e','r','d','f','c','v' para cambiar la rotacion, asegurate de que el cuadrado este "dentro" de tu pierna.  

Felicitaciones, los trackers deberian estar funcionando.  

Si encuentras que algo se ve mal, puedes presionar '7' para activar el modo debug, esto abrira una aplicacion que te dira mas informacion, en la primera pantalla se vera la posicion final de los trackers "Final View",  
si presionas cualquier tecla, esta cambiara de pantalla, la siguiente es "Raw Tackers", esta te mostrara las posicion reales de los tags, y tambien de los que fueron registrados (util mientras calibras las camaras, entre mas cerca el rojo este con el verde, mejor), la siguiente es "Combined and predicted" este te mostrara el centro que tu les diste de 'posZ' en 'trackers.txt' y tambien su orientacion,  
finalmente esta Adjust Exceptions range que es no es importante





