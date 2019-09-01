#Fuentes Útiles

###Manejo de Branches (Ramas)
La idea es que utilicemos branches lo más posible para mantener una estructura de versionamiento estable.

Por default, siempre vamos a estar posicionados en la rama origin/master, que será el tronco principal del versionamiento del código. En esta rama se mantendrá únicamente aquel código que ya ha sido testeado y del cual mantendremos siempre una versión funcional.

Para crear y utilizar otras branches, deberemos utilizar los siguientes comandos:

$ git branch nuevafuncionalidad
$ git checkout nuevafuncionalidad
El primero lo que hace es crear esa nueva branch a utilizar, y con el segundo nos posicionamos sobre ella.

Para verificar la branch en la que se encuentran, pueden utilizar:

$ git branch
Una vez se encuentran en la nueva rama, pueden empezar a implementar y commitear a discreción.

Recuerden que pueden volver a la rama master o cambiar a cualquier otra rama ya existente con el comando de git checkout rama.

Realizar Merge
Cuando vean que ya han implementado totalmente la funcionalidad, van a tener que realizar un merge con la branch de origin/master. Para eso se pasan a la rama Master y luego mandan el comando de merge:

$ git checkout master
$ git merge nuevafuncionalidad
Esto puede o no presentar conflictos de merge que deberán resolverse manualmente. Los conflictos suelen aparecer únicamente en los casos en los que se modifican los mismos archivos sobre la misma línea. El mejor consejo es tratar de no tocar los mismos archivos, pero en casos como el de aplicacion.kt eso puede llegar a ser menos que imposible.

Conflictos de Mergeo
Imaginen un caso hipotético donde desean mergear una rama y les tira un error:

$ git merge otrarama
Auto-merged archivo.bleh
CONFLICT (content): Merge conflict in archivo.bleh
Automatic merge failed; fix conflicts and then commit the result.
Ahí está dando la información de la fuente del conflicto en el archivo archivo.bleh, por lo que hay que abrirlo y buscar la línea culpable. Es probable encontrarse con algo similar a esto (Si fuera un archivo de python):

    def send_delivery_request(request):
        self._server.request = request
&#60; &#60;&#60;&#60;&#60;&#60;&#60; HEAD:deliverance.py
        self._rtime = 6000
===
        self._rtime = get_time_to_live(request)
&#62;&#62;&#62;&#62;&#62;&#62;&#62; otrarama:deliverance.py
        # Send the request to the service
        self._server.send(self._rtime)
Acá vemos que en la misma línea donde se quiere referenciar la variable self._rtime = se alteró en dos versiones. Allí hay que decidir manualmente cuál es la versión que debe permanecer, que en este caso es aquella más desarrollada, la de get_time_to_live(request)

    def send_delivery_request(request):
        self._server.request = request
        self._rtime = get_time_to_live(request)
        # Send the request to the service
        self._server.send(self._rtime)
Una vez solucionado este problema, sencillamente agregamos los archivos en conflicto y hacemos un nuevo commit:

$ git add deliverance.py
$ git commit -m &#39;solucionando conflictos&#39;
Created commit 17c688a: solucionando conflicto
Links
Fuente de datos: https://www.genbeta.com/desarrollo/manejo-de-ramas-de-desarrollo-con-git

Y sino, cualquier cosa me consultan :P