;; Definición de plantillas
(deftemplate objeto
   (slot nombre)
   (slot ubicacion-actual))

(deftemplate habitacion
   (slot nombre)
   (slot centro))

(deftemplate robot
   (slot ubicacion-actual))

(deftemplate objetivo
   (slot objeto)
   (slot habitacion-destino))

;; Definición de funciones
(deffunction mover-robot (?nueva-ubicacion)
   (bind ?robot (find-all-facts ((?r robot)) TRUE))
   (if (> (length ?robot) 0) then
      (retract (nth 1 ?robot))
      (assert (robot (ubicacion-actual ?nueva-ubicacion)))
      (printout t "Robot movido a " ?nueva-ubicacion crlf)))

(deffunction mover-objeto (?nombre ?nueva-ubicacion)
   (bind ?objeto (find-all-facts ((?o objeto)) (eq ?o:nombre ?nombre)))
   (if (> (length ?objeto) 0) then
      (retract (nth 1 ?objeto))
      (assert (objeto (nombre ?nombre) (ubicacion-actual ?nueva-ubicacion)))
      (printout t ?nombre " movido a " ?nueva-ubicacion crlf)))

;; Hechos iniciales
(assert (objeto (nombre "caja1") (ubicacion-actual "almacen")))
(assert (objeto (nombre "caja2") (ubicacion-actual "oficina")))
(assert (objeto (nombre "caja3") (ubicacion-actual "almacen")))

(assert (habitacion (nombre "almacen") (centro (x 0) (y 0))))
(assert (habitacion (nombre "oficina") (centro (x 10) (y 10))))
(assert (habitacion (nombre "sala") (centro (x 20) (y 20))))

(assert (robot (ubicacion-actual "almacen")))

(assert (objetivo (objeto "caja1") (habitacion-destino "oficina")))
(assert (objetivo (objeto "caja3") (habitacion-destino "sala")))

;; Regla para mover el robot a la ubicación del objeto
(defrule mover-robot-al-objeto
   ?r <- (robot (ubicacion-actual ?ubicacion-robot))
   ?o <- (objeto (nombre ?nombre-objeto) (ubicacion-actual ?ubicacion-objeto))
   (objetivo (objeto ?nombre-objeto) (habitacion-destino ?destino))
   (test (neq ?ubicacion-robot ?ubicacion-objeto))
   =>
   (mover-robot ?ubicacion-objeto))

;; Regla para mover el objeto a la ubicación de destino
(defrule mover-objeto-al-destino
   ?r <- (robot (ubicacion-actual ?ubicacion-robot))
   ?o <- (objeto (nombre ?nombre-objeto) (ubicacion-actual ?ubicacion-objeto))
   ?obj <- (objetivo (objeto ?nombre-objeto) (habitacion-destino ?destino))
   (test (eq ?ubicacion-robot ?ubicacion-objeto))
   (test (neq ?ubicacion-objeto ?destino))
   =>
   (mover-objeto ?nombre-objeto ?destino)
   (mover-robot ?destino))

;; Ejecutar las reglas
(reset)
(run)



