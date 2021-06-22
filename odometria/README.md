implementación en Webots de una arquitectura híbrida que añada aparte de comportamientos similares a los realizados en el controlador subsumido,una funcionalidad de mapeo y de localización, así como un comportamiento de planificación de trayectorias.

Con dicha implementación, el objetivo del robot será encontrar un objeto de color verde, cuya posición puede cambiar cada vez que el robot lo busca, en un entorno discreto, de dimensiones conocidas y cuya forma es desconocida.
La ejecución se realizará en 2 etapas:
1. Una etapa inicial de exploración del entorno y creación del mapa.
2. Una segunda etapa en la que se utiliza el mapa para buscar el objeto y regresar al punto de origen de forma eficiente.
3. En la fase de creación del mapa, se añadirá una funcionalidad para actualización de un mapa mientras el robot explora el entorno siguiendo las paredes.
En la fase de búsqueda del objeto, el robot seguirá explorando el entorno siguiendo las paredes, aunque esta vez mediante el mapa creado anteriormente. Una vez encuentre el objeto, se añadirá un comportamiento de planificación de trayectorias para que el robot regrese de forma eficiente a la base.
El mapa creado será un grid de ocupación binario que será actualizado de acuerdo a la información obtenida con los infrarrojos, que nos permitirán conocer si las celdas adyacentes a la del robot están libres u ocupadas.
Para localizar al robot en el mapa se utilizará odometría, usando los encoders de los motores para avanzar celda a celda y hacer giros precisos múltiplos de 90º para así poder conocer las coordenadas de posición en el entorno.
