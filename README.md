# grupo_5

### Integrantes:

| Nombre y Apellido              |      Mail                      |     usuario Gitlab   |
| -----------------------------  | ------------------------------ | -------------------  |
|  Svrznjak Spinelli Francisco   | franciscospinelli98@gmail.com  | FranSpinelli         |
| Liotine Cristian Ezequiel      | cris.eze.99@gmail.com          | Cristian2301         |
| Franco Santos                  |  francosantos874@gmail.com     | fransantt0s          |

## Entregas:

### Práctica 1:  
	- Ok.

### Práctica 2:
	- Ok, funciona.

Detalles menores 

Esta bien usar la clase `Queue`, aunque quizas no es necesario. La clase `Queue` tiene mecanismos de bloqueo, control de tamaño, sincronizacion entre consumidores y productores. 
No es un problema, pero excede las necesidades de nuestra ReadyQueue.
Por otro lado, dado que la estan creando de tamaño infinito, no es necesario utilizar `put_nowait()` y `get_nowait()`. Con `put()` y `get()` alcanza. (y queda mas claro)

Tambien podrian haber usado `empty()` para determinar si la cola esta vacia, en lugar de [preguntar si el tamaño es cero](https://gitlab.com/so-unq-2019-s2-c1/grupo_5/blob/master/practicas/practica_2/so.py#L71)

de todas maneras el trabajo esta muy bien, son detalles menores que no es necesario corregir.