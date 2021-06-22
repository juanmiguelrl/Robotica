El objetivo consiste en la implementación en Webots de un comportamiento en el que el robot busque y se dirija hacia un objeto de color verde en un entorno donde dicho robot no ve el objetivo inicialmente.
Mediante arquitectura subsumida y sin utilizar mapas ni conocimientos a priori del entorno.

Para obtener el comportamiento final del robot, se complementarán entre sí los siguientes comportamientos con una relación jerárquica:
Mayor Prioridad: Escapar
2ª Prioridad: Ir hacia objeto
3ª Prioridad: Seguir pared
4ª Prioridad: Acercarse a pared
Acercarse a pared: Comportamiento basado en buscar una pared y aproximarse a ella.
Seguir pared: Utilizando sensores de distancia, al acercarse a un objeto, seguir su contorno, dejándolo a la izquierda.
Ir hacia objeto: Utilizando la cámara, detectar el objeto por color (que en este caso será verde). Cuando el objeto se encuentra a lo lejos, el robot se acercará controladamente hacia él y cuando esté bastante cerca, se detendrá y finalizará el comportamiento.
Escapar: Comportamiento que se ejecuta cuando el robot ve bloqueado su avance
