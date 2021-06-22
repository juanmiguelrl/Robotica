Implementación en Webots, un robot khepera IV aprenda el comportamiento de seguir una línea negra coloreada en el suelo. Además, se añade un control de evitar obstáculos.
Dicho comportamiento se consigue mediante aprendizaje por refuerzo, más concretamente por medio de Q-learning.

Los estados definidos son 6 posibles estados en los que se puede encontrar el robot:
1. Pared cerca por la izquierda.
2. Pared cerca por delante.
3. Pared cerca por la derecha.
4. Abandona línea por la izquierda
5. Abandona línea por la derecha
6. Resto de casos

Las acciones definidas son:
1. Girar izquierda
2. Girar derecha
3. Avanzar
