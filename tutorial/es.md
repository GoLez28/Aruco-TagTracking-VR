Primero, debo decir que mi programa no esta completo, si quieres una solucion mas refinada, moderna y facil de usar utiliza este proyecto https://github.com/ju1ce/April-Tag-VR-FullBody-Tracker,  
pero solo utiliza una camara,  
si vas a querer utilizar mi programa, vas a tener que descargar ese proyecto de todas formas, por tema de calibrar la camara

Empezando, debes instalar un driver para steamvr, que permitira que el tracker sea detectado por steamvr,  
debes instalar VMT https://github.com/gpsnmeajp/VirtualMotionTracker/releases/tag/v0.15 , el instalador es bastante facil de usar,  
una vez instalado no se necesita hacer nada mas (eso creo).
Despues, vas a tener que descargar mi programa, cuando lo abras por primera vez, se generara el archivo "arucoboard.png" al lado del .exe,  
esto es lo que vas a tener que imprimir, pero solo lo que necesites, por ejemplo si vas a usar 4 para el pie derecho, 4 pal derecho y 6 para la cintura,  
necesitaras 14 en total, lo demas va a ser innecesario.

asegurate de que todos los tags sean del mismo tamaño, cuando ya los tengas impresos, midelos y coloca los milimetros del cuadrado negro en config.txt, trackerSize=x

pegas los tags impresos en un pedazo de carton o cualquier cosa que sirva de tal forma

esta es la forma que tengo puesto los trackers

pero tambien puedes colocarlos asi, o realmente de cualquier forma,  
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

Para las camaras yo estoy utilizando celulares por la app iVCam (si quieres saber como configurarlo de la mejor forma, me preguntas),  
pero primero hay que calibrar las camaras, para eso necesitamos el otro proyecto 'Juices VR Marker Tracking', por que ahi tiene una opcion para calibrar camaras,  
al abrir esa aplicacion, te vas a la pestaña Params, y en 'ID of camera' colocas 0, abajo le das Save,  
te ve vas a la pestaña de Camera, le das Start, y Preview, si en la camara no se ve nada, tienes que cambiar de camara, le das Stop y te vas a Params y pones otro numero en ID of camera y repites,  
cuando el preview muestre la camara que quieres calibrar, en la pestaña de Camera, le das a Calibrate camera,  
en la carpeta dela app de calibracion, deberia haber una carpeta llamada 'images-to-print' con 'charuco_board.jpg',  
se supone que deberias imprimir eso tambien pero simplemente lo pones en pantalla completa y funciona igual,  
la camara lo pones de manera que se vea la imagen completa, y vas moviendo la camara en varias posiciones, hasta que encuentra que sea suficiente,  

Una vez hecho eso en la carpeta de la app de calibracion, deberia haber un archivo llamado 'calib.yaml' en la carpeta 'config',  
copias los valores del archivo 'config.yaml' a 'cameraParameters1.xml' que esta en la carpeta de la aplicacion que vamos a usar,  
hay que copiar de camMat a cameraMatrix, y de distCoeffs a dist_coeffs, fijate que xml no usa comas si no que espacios,  
(si la camara cambia en tema de resolucion, o otra cosa, se debe hacer la calibracion de nuevo)

ahora toca configurar las camaras, en config.txt tienes que asegurarte de que cameraXFile sea el nombre del xml de calibracion (un xml para cada camara), seguido de la resolucion y ID (Index),  
lo mismo para todas las camaras que vas a usar definido por totalCameras (entre mas camaras, resolucion, fps, mas CPU va a usar),  
en mi parecer 960p en 4:3 es lo mejor para una camara, 600p es pasable, pero 480p es simplemente muy impreciso

