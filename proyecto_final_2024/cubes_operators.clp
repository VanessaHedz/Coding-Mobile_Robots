;;;============================================================================
;;;   Programa del Mundo de los Bloques
;;;   Para ejecutarlo, solamente carguelo en CLIPS: clips -f cubes_operators.clp 
;;;   de (reset) y ejecutelo con (run)
;;;   Los commandos que se le daran al robot quedan en el archivo commnands.dat
;;;============================================================================


(deftemplate on-top-of
	(slot upper)
	(slot lower)
)

(deftemplate actualRoom (slot room))

(deftemplate graspObject (slot move))

(deftemplate goal (slot move)(slot on-top-of))
(deftemplate goals (slot move)(slot room))


(deffacts initial-state
	(block A)
	(block B)
	(block C)
	(block D)
	(block E)
	(block F)
	(on-top-of (upper nothing)(lower A))
	(on-top-of (upper A)(lower B))
	(on-top-of (upper B)(lower C))
	(on-top-of (upper C)(lower floor))
	(on-top-of (upper nothing)(lower D))
	(on-top-of (upper D)(lower E))
	(on-top-of (upper E)(lower F))
	(on-top-of (upper F)(lower floor))
	;(goal (move C)(on-top-of E))
	;(goal (move F)(on-top-of C))
	(actualRoom (room deposit))
	(goals (move F)(room studio))
	(goals (move E)(room bedroom))
	(goals (move D)(room corridor))
	(goals (move C)(room deposit))
	(goals (move B)(room kitchen))
	(goals (move A)(room service))
)



; ia opens the file to save the robot commands
(defrule open-file
	(declare (salience 100))
        ?f <- (initial-fact)

        =>
        (retract ?f)
        (open commands.dat output-file "w")
        (assert (file-commands output-file))
	(assert (file-name commands.dat))
)


(defrule grasp-object
	(graspObject (move ?block))
	(on-top-of (upper nothing) (lower ?block))
	?stack <- (on-top-of (upper ?block) (lower ?block2))
	(file-commands ?file)
	=>	
	(retract ?stack)
	(assert (on-top-of (upper nothing)(lower ?block2)))
	(printout t ?block " its grasped by the robot " crlf)
	(printout ?file "OPERATOR grasp " ?block " "  crlf)
	(assert (Grasped))
)

(defrule relase-object
	?goal <- (goals (move ?block)(room ?Room))
	(block ?block)
	(Room (name ?Room))
	?mov <- (moving)
	?gra <- (Grasped)
	?grasped <- (graspObject (move ?block))
	?item <- (item (name ?block)(room ?itemRoom))
	?currentRoomF <- (actualRoom (room ?currentRoom))
	(test (eq ?Room ?currentRoom))
	(file-commands ?file)
	=>
	(retract ?grasped ?mov ?goal ?gra)
	(modify ?item (room ?currentRoom))
	(printout ?file "OPERATOR release " ?block " "  crlf)
	
)

(defrule go-to-room
	(goals (move ?block)(room ?Room))
	(block ?block)
	(Room (name ?Room)(center ?x ?y))
	(graspObject (move ?block))
	?item <- (item (name ?block)(room ?itemRoom))
	?currentRoomF <- (actualRoom (room ?currentRoom))
	(file-commands ?file)
	=>
	(retract ?currentRoomF)
	(assert (actualRoom (room ?Room)))
	(assert (moving))
	(printout ?file "OPERATOR go to " ?Room " " ?x "," ?y  crlf)
)




(defrule find-object
	(goals (move ?block)(room ?Room))
	(block ?block)
	(Room (name ?Room))
	(not (Grasped))
	(on-top-of (upper nothing) (lower ?block))
	?item <- (item (name ?block)(room ?itemRoom))
	?currentRoomF <- (actualRoom (room ?currentRoom))
	(test (eq ?itemRoom ?currentRoom))
	(file-commands ?file)
	=>
	(printout t ?block " is in the same room: " ?itemRoom crlf)
	(assert (graspObject (move ?block)))
	(printout ?file "OPERATOR find " ?block " "  crlf)
)

(defrule find-object-go-to-room
	(goals (move ?block)(room ?Room))
	(block ?block)
	(Room (name ?Room))
	?item <- (item (name ?block)(room ?itemRoom))
	?Roomi <- ( Room (name ?itemRoom)(center ?x ?y))
	?currentRoomF <- (actualRoom (room ?currentRoom))
	(test (neq ?itemRoom ?currentRoom))
	(file-commands ?file)
	=>
	(printout t ?block " isn't in the same room: " ?itemRoom crlf)
	(retract ?currentRoomF)
	(assert (actualRoom (room ?itemRoom)))
	(printout ?file "OPERATOR go to " ?itemRoom " " ?x "," ?y  crlf)
)





(defrule move-directly
	?goal <- (goal (move ?block1) (on-top-of ?block2))
	(block ?block1)
	(block ?block2)
	(on-top-of (upper nothing) (lower ?block1))
	?stack-1 <- (on-top-of (upper ?block1)(lower ?block3))
	?stack-2 <- (on-top-of (upper nothing)(lower ?block2))
	(file-commands ?file)
	=>
	(retract ?goal ?stack-1 ?stack-2)
	(assert (on-top-of (upper ?block1)(lower ?block2))
       	  	(on-top-of (upper nothing)(lower ?block3)))
	(printout t ?block1 " moved on top of " ?block2 "." crlf)
	(printout ?file "OPERATOR move " ?block1 " " ?block2  crlf)
)


(defrule move-to-floor
	?goal <- (goal (move ?block1) (on-top-of floor))
	(block ?block1)
	(on-top-of (upper nothing) (lower ?block1))
	?stack <- (on-top-of (upper ?block1) (lower ?block2))	
	(file-commands ?file)
	=>
	(retract ?goal ?stack)
	(assert (on-top-of (upper ?block1)(lower floor))
        	(on-top-of (upper nothing)(lower ?block2)))
	(printout t ?block1 " moved on top of the floor. " crlf)
	(printout ?file "OPERATOR move " ?block1 " floor" crlf)
)


(defrule clear-upper-block
	(goal (move ?block1))
	(block ?block1)
	(on-top-of (upper ?block2) (lower ?block1))
	(block ?block2)
	=>
	(assert (goal (move ?block2)(on-top-of floor)))
)




(defrule clear-lower-block
	(goal (on-top-of ?block1))
	(block ?block1)
	(on-top-of (upper ?block2) (lower ?block1))
	(block ?block2)
	=>
	(assert (goal (move ?block2)(on-top-of floor)))
)



(defrule finish-planning
        (declare (salience -400))
        ?f <- (file-commands ?file)
	(file-name ?name)
        =>
        (retract ?f)
        (printout t crlf "finish planner" crlf)
	(printout t "Commands in file " ?name crlf)
        (printout ?file "finish planner" crlf)
        (close ?file)

)
