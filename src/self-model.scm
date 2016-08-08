;
; self-model.scm
;
; Model of Eva's current physical state, represented in the Atomspace.
;
; This attempts to maintain a model, in the atomspace, of what the robot
; is actually doing, from moment to moment.  This model is important for
; multiple reasons:
;
; -- It is needed for Action Orchestration, to make sure that multiple
;    conflicting command sources do not cause the robot to do incoherent
;    things (such as smile and frown at the same time, or move lips
;    without talking, etc.)
;
; -- It is needed for self-awareness, so that the chatbot can respond to
;    questions about what Eva is doing.
;
; Example usage:
; Load the needed modules.
; (use-modules (opencog) (opencog query) (opencog exec))
; (use-modules (opencog atom-types) (opencog python))
; (use-modules (opencog eva-model))
;;;; start roscore before doing the load below.
; (python-eval "execfile('atomic.py')")
;
; Examples and debugging hints:
; Some (but not all) state queries:
; (cog-evaluate! (DefinedPredicate "chatbot is talking?"))
; (cog-evaluate! (DefinedPredicate "chatbot is listening?"))
; (cog-evaluate! (DefinedPredicate "chatbot is happy"))
; (cog-evaluate! (DefinedPredicateNode "Did someone arrive?"))
; (cog-evaluate! (DefinedPredicateNode "Someone visible?"))
; (cog-execute! (DefinedSchemaNode "Num visible faces"))
;
(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog query) (opencog exec))
(use-modules (opencog atom-types))

; ------------------------------------------------------
; State variables
; XXX FIXME There are a bunch of define-publics in here, they probably
; should not be; they're needed only by the behavior module.

; Face tracking state indicates if we respond to face tracking events.
; Other than face tracking events there speech events to respond
(define-public face-tracking-state (AnchorNode "Face Tracking State"))
(define-public face-tracking-on (ConceptNode "FaceTrackingOn"))
(define-public face-tracking-off (ConceptNode "FaceTrackingOff"))
;; Facetracking is enabled by default
(StateLink face-tracking-state face-tracking-on)

; Soma: awake, agitated, excited, tired, manic, depressed, in-pain.
; See, however, "affects", below.
(define-public soma-state (AnchorNode "Soma State"))
(define-public soma-sleeping (ConceptNode "Sleeping"))
(define-public soma-awake (ConceptNode "Awake"))
(define-public soma-bored (ConceptNode "Bored"))

;; Assume Eva is sleeping at first
(StateLink soma-state soma-sleeping)

;; True if sleeping, else false.
(DefineLink
	(DefinedPredicate "Is sleeping?")
	(Equal (SetLink soma-sleeping)
		(Get (State soma-state (Variable "$x")))))

;; True if bored, else false
(DefineLink
	(DefinedPredicate "Is bored?")
	(Equal (SetLink soma-bored)
		(Get (State soma-state (Variable "$x")))))

; -----------
; The "emotional state" of the robot.  Corresponds to states declared
; in the `cfg-*.scm` file.
;
; XXX this should be renamed "current facial expression", see the
; README-affects.md for general discussion.
(define-public emotion-state (AnchorNode "Emotion State"))
(define-public emotion-neutral (ConceptNode "neutral"))

(StateLink emotion-state emotion-neutral)

; -----------
;; The eye-contact-state will be linked to the face-id of
;; person with whom we are making eye-contact with.
;; This is usually the same as the interaction-state, but not always:
;; If told to look away, she will break eye-contact but not break
;; interaction.
(define-public eye-contact-state (AnchorNode "Eye Contact State"))
(define-public no-interaction (ConceptNode "none"))

(StateLink eye-contact-state no-interaction)

; The face to glance at.
(define-public glance-state (AnchorNode "Glance State"))
(StateLink glance-state no-interaction)

;; Linked to face-id that needs immediate interaction.
;; Currently it is set from ROS
(define request-eye-contact-state (AnchorNode "Request Interaction"))
(StateLink request-eye-contact-state no-interaction)

;; The "look at neutral position" face. Used to tell the eye/head
;; movement subsystem to move to a neutral position.
(define neutral-face (ConceptNode "0"))

;; The person she is interacting with.
;; Not the same as eye-contact state, because she may have been
;; told to look the other way (i.e. break off eye contact)
(define-public interaction-state (AnchorNode "Interaction State"))
(StateLink interaction-state no-interaction)

; --------------------------------------------------------
; Chatbot-related stuff.  In the current design, the chatbot talks
; whenever it feels like it; we are simply told when it is talking
; when it has stopped talking, and what emotions we should display,
; so that it's consistent with the speech emotions.

; Chat state. Is the robot talking (vocalizing), or not, right now?
; NB the python code in put_atoms.py uses these defines!
; This is a state-machine, valid transitions are:
; listening -> started talking
; started talking -> talking
; talking -> stoped talking
; stopped talking -> listening.
(define-public chat-state (AnchorNode "Chat State"))
(define-public chat-listen (ConceptNode "Listening"))
(define-public chat-listen-start (ConceptNode "Listening Start"))
(define-public chat-listen-stop (ConceptNode "Listening Stop"))
(define-public chat-start  (ConceptNode "Start Talking"))
(define-public chat-talk   (ConceptNode "Talking"))
(define-public chat-stop   (ConceptNode "Stop Talking"))
(define-public chat-idle   (ConceptNode "Chat inactive"))
; Wait for speech events to trigger speech behaviors
(StateLink chat-state chat-idle)

(DefineLink
	(DefinedPredicate "chatbot started talking?")
	(Equal (Set chat-start)
		(Get (State chat-state (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot started listening")
	(Equal (Set chat-listen-start)
		(Get (State chat-state (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot is talking?")
	(Equal (Set chat-talk)
		(Get (State chat-state (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot stopped talking?")
	(Equal (Set chat-stop)
		(Get (State chat-state (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot started listening?")
	(Equal (Set chat-listen-start)
		(Get (State chat-state (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot is listening?")
	(Equal (Set chat-listen)
		(Get (State chat-state (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot stopped listening?")
	(Equal (Set chat-listen-stop)
		(Get (State chat-state (Variable "$x")))))

; Chat affect. Is the robot happy about what its saying?
; Right now, there are only two affects: happy and not happy.
; NB the python code uses these defines!
; XXX FIXME: Note also: we currently fail to distinguish the affect
; that was perceived, from our own state. There is a ROS message that
; informs us about what the perceived affect was: it sets this state.
;
(define-public chat-affect (AnchorNode "Chat Affect"))
(define-public chat-happy (ConceptNode "Happy"))
(define-public chat-negative (ConceptNode "Negative"))
(StateLink chat-affect chat-happy)

(DefineLink
	(DefinedPredicate "chatbot is happy")
	(Equal
		(Set chat-happy)
		(Get (State chat-affect (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot is negative")
	(Equal
		(Set chat-negative)
		(Get (State chat-affect (Variable "$x")))))

; --------------------------------------------------------
; Speech-to-text (STT) related stuff.
; If the STT system hears something, it sends us the text string.
; Handle it here.

(define heard-sound (Anchor "Heard Something Recently"))
(define heard-nothing (SentenceNode ""))
(State heard-sound heard-nothing)

;; Process text that was "heard" (e.g. from the STT module)
;; This is a function call, with one argument: a SentenceNode.
(DefineLink
	(DefinedPredicate "heard text")
	(LambdaLink
		(Variable "$text")
		(SequentialAnd
			(Evaluation (GroundedPredicate "scm: dispatch-text")
				(ListLink (Variable "$text")))

			; Set timestamp for when something was last heard.
			(True (DefinedSchema "set heard-something timestamp"))

			; "heard-sound" is used to wake her up, if sleeping.
			(True (Put
					(State heard-sound (Variable "$noise"))
					(Variable "$text")))
		)
	)
)

;; Return true if something was heard (recently).
;; This can be used only once: it clears the state immediately, so
;; if asked a second time, nothing was heard.
(DefineLink
	(DefinedPredicate "Heard Something?")
	(SequentialAnd
		(NotLink (Equal (SetLink heard-nothing)
			(Get (State heard-sound (Variable "$x")))))
		(True (Put (State heard-sound (Variable "$x")) heard-nothing))
	))

;; Loud sound value.
(define loud-sound  (AnchorNode "Sudden sound change value"))
(State loud-sound (Number 0)) ; There isn't any sudden change in sound Decibel

;; Return true if a loud voice is heard
(DefineLink
	(DefinedPredicate "Heard Loud Voice?")
	(GreaterThan
		(Get (State loud-sound (Variable "$x")))
		(Number 0)))

; --------------------------------------------------------
; Time-stamp-related stuff.

;; Define setters and getters for timestamps. Perhaps this should
;; be replaced by the timeserver??

(define (timestamp-template name)

	; The name of state node holding the timestamp.
	(define ts-name (string-append "start-" name "-timestamp"))
	(define prev-ts (string-append "previous-" name "-call"))

	; The state node actually holding the timestamp.
	(State (Schema ts-name) (Number 0))

	; timestamp setter
	(DefineLink
		(DefinedSchema (string-append "set " name " timestamp"))
		(Put (State (Schema ts-name) (Variable "$x")) (TimeLink)))

	; timestamp getter
	(DefineLink
		(DefinedSchema (string-append "get " name " timestamp"))
		(Get (State (Schema ts-name) (Variable "$x"))))

	; Additional state, used for computing integral, for probabilities.
	; See below, in the time-to-change template.
	(State (Schema prev-ts) (Number 0))
)

; "interaction" -- record the start time of an interaction.
; defines (DefinedSchema "set interaction timestamp") etc.
(timestamp-template "interaction")

; "expression" -- time when a new expression started being shown.
(timestamp-template "expression")
; "gesture" -- time when the last gesture was made.
(timestamp-template "gesture")

; "bored" -- when Eva last got bored.
(timestamp-template "bored")
; "sleep" -- when Eva fell asleep.
(timestamp-template "sleep")

; "attn-search" -- when Eva started searching for attention.
(timestamp-template "attn-search")

; "heard-something" -- when Eva heard a sentence from STT.
(timestamp-template "heard-something")

; --------------------------------------------------------
; Some debug prints.

(define (print-msg node) (display (cog-name node)) (newline) (stv 1 1))
(define (print-atom atom) (format #t "~a\n" atom) (stv 1 1))

; --------------------------------------------------------
; Basic utilities for working with newly-visible faces.

;; ------
;;
;; Return true if a new face has become visible.
;; A "new  face" is one that is visible (in the atomspace) but
;; has not yet been acked.  Acking usually occurs when we make
;; eye-contact with them.
(DefineLink
	(DefinedPredicateNode "Did someone arrive?")
	(SatisfactionLink
		(AndLink
			; If someone is visible...
			(PresentLink (EvaluationLink (PredicateNode "visible face")
					(ListLink (VariableNode "$face-id"))))
			; but not yet acknowledged...
			(AbsentLink (EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id"))))
		)))

;; Return the set of newly-arrived faces.
;; Will return a SetLink holding zero, one or more face id's
(DefineLink
	(DefinedSchemaNode "New arrivals")
	(GetLink
		(AndLink
			; If someone is visible...
			(PresentLink (EvaluationLink (PredicateNode "visible face")
					(ListLink (VariableNode "$face-id"))))
			; but not yet acknowledged...
			(AbsentLink (EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id"))))
	)))

;; Return the person with whom we are currently interacting.
(DefineLink
	(DefinedSchema "Current interaction target")
	(Get (State interaction-state (Variable "$x"))))

;; Return true if some face has is no longer visible (has left the room)
;; We detect this by looking for "acked" faces tat are not also visible.
(DefineLink
	(DefinedPredicateNode "Did someone leave?")
	(SatisfactionLink
		(TypedVariable (Variable "$face-id") (Type "NumberNode"))
		(AndLink
			; If someone was previously acked...
			(PresentLink (EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id"))))
			; But is no loger visible...
			(AbsentLink (EvaluationLink (PredicateNode "visible face")
					(ListLink (VariableNode "$face-id"))))
		)))

;; Return list of recetly departed individuals
(DefineLink
	(DefinedSchema "New departures")
	(Get
		(TypedVariable (Variable "$face-id") (Type "NumberNode"))
		(AndLink
			; If someone was previously acked...
			(PresentLink (Evaluation (Predicate "acked face")
					(ListLink (Variable "$face-id"))))
			; But is no loger visible...
			(AbsentLink (Evaluation (Predicate "visible face")
					(List (Variable "$face-id"))))
	)))

;;
;; Was the the room empty, viz: Does the atomspace contains the link
;; (StateLink (AnchorNode "Room State") (ConceptNode "room empty"))?
;; Note that the room state is updated only when "Update room state"
;; is called, so faces may be visible, but the room marked as empty.
;; Think "level trigger" instead of "edge trigger".
(DefineLink
	(DefinedPredicateNode "was room empty?")
	(EqualLink
		(SetLink room-empty)
		(GetLink (StateLink room-state (VariableNode "$x")))
	))

;; Is there someone present?  We check for acked faces.
;; The someone-arrived code converts newly-visible faces to acked faces.
(DefineLink
	(DefinedPredicateNode "Someone visible?")
	(SatisfactionLink
		(TypedVariable (Variable "$face-id") (Type "NumberNode"))
		(PresentLink
			(EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id")))
		)))

;; Return the number of visible faces
(DefineLink
	(DefinedSchema "Num visible faces")
	(Arity
		(Get
			(TypedVariable (Variable "$face-id") (Type "NumberNode"))
			(Evaluation (Predicate "acked face")
				(ListLink (Variable "$face-id"))))))

; True if more than one face is visible.
(DefineLink
	(DefinedPredicateNode "More than one face visible")
	(GreaterThanLink
		(DefinedSchemaNode "Num visible faces")
		(NumberNode 1)))


;; Randomly select a face out of the crowd.
(DefineLink
	(DefinedSchema "Select random face")
	(RandomChoice (Get
		(TypedVariable (Variable "$face-id") (Type "NumberNode"))
		(Evaluation (Predicate "acked face")
			(ListLink (Variable "$face-id")))
	)))

;; Randomly glance at someone (who we are not currently making
;; eye-contact with)
(DefineLink
	(DefinedPredicate "Select random glance target")
	(SequentialAnd
		; Recursive loop, keep picking, while the current glance target
		; is the same as the current interaction target.
		(TrueLink
			(PutLink (StateLink glance-state (VariableNode "$face-id"))
				(DefinedSchemaNode "Select random face")))
		(EqualLink
			(GetLink (StateLink glance-state (VariableNode "$face-id")))
			(GetLink (StateLink eye-contact-state (VariableNode "$face-id")))
		)
		(DefinedPredicateNode "More than one face visible")
		(DefinedPredicateNode "Select random glance target")
	))

;; Update the room empty/full status; update the list of acknowledged
;; faces.
(DefineLink
	(DefinedPredicate "Update status")
	(SequentialAnd
		(DefinedPredicate "Update room state")
		; If there was more than one face that recently arrived, then
		; we assume that this face was selected as the eye-contact
		; target.  We convert this to an "acked" face.  Other
		; newly-arrived faces stay "newly arrived" (and non-acked)
		; until ... until they are eye-contacted.
		(True (Put
				(Evaluation (Predicate "acked face")
						(ListLink (Variable "$face-id")))
				(Get (State eye-contact-state (Variable "$x")))))
	))

;; Remove the lost faces from "acked face" (so that "acked face" accurately
;; reflects the visible faces)
(DefineLink
	(DefinedPredicateNode "Clear lost face")
	(TrueLink (PutLink
		(DeleteLink
			(EvaluationLink (PredicateNode "acked face")
				(ListLink (VariableNode "$face-id"))))
		(DefinedSchemaNode "New departures"))
	))
;; ------
;;
;; Return true if interacting with someone.
;; This is a compound predicate: we are interacting if the interaction
;; state is set, or if the TTS system/chatbot is still vocalizing.
;; (cog-evaluate! (DefinedPredicateNode "Is interacting with someone?"))
(DefineLink
	(DefinedPredicate "Is interacting with someone?")
	(OrLink
		; true if talking not listening.
		(NotLink (DefinedPredicate "chatbot is listening?"))
		; true if not not-making eye-contact.
		(NotLink (Equal
			(SetLink no-interaction)
			(Get (State interaction-state (Variable "$x"))))
	)))


;; Return true if someone requests interaction.  This person will
;; become the new focus of attention.
(DefineLink
	(DefinedPredicate "Skip Interaction?")
	(Equal
		(SetLink face-tracking-off)
		(Get (State face-tracking-state (Variable "$x"))))
	)

;; Return true if someone requests interaction.  This person will
;; become the new focus of attention.
(DefineLink
	(DefinedPredicate "Someone requests interaction?")
	(NotLink (Equal
		(SetLink no-interaction)
		(Get (State request-eye-contact-state (Variable "$x"))))
	))

;; Send ROS message to actually make eye-contact with the person
;; we should be making eye-contact with.
(DefineLink
	(DefinedPredicate "look at person")
	(SequentialAnd
		;; Issue the look-at command, only if there is someone to
		;; make eye conact with.
		(NotLink (Equal
			(Get (State eye-contact-state (Variable "$x")))
			(SetLink no-interaction)))
		(True (Put
			(Evaluation (GroundedPredicate "py:look_at_face")
				(ListLink (Variable "$face")))
			(Get (State eye-contact-state (Variable "$x")))))
	))

;; Break eye contact; this does not change the interaction state.
(DefineLink
	(DefinedPredicate "break eye contact")
	(True (Put (State eye-contact-state (Variable "$face-id"))
		no-interaction))
)

;; Make eye-contact with the interaction target.
(DefineLink
	(DefinedPredicate "make eye contact")
	(True (Put (State eye-contact-state (Variable "$face-id"))
		(Get (State interaction-state (Variable "$x"))) ))
)

;; Move to a neutral head position.
;; This clears the interaction and eye-contact state,
;; and turns head to face forward.
(DefineLink
	(DefinedPredicate "return to neutral")
	(SequentialAnd
		(Evaluation (GroundedPredicate "py:look_at_face")
			(ListLink neutral-face))
		(True (Put
			(State eye-contact-state (Variable "$face-id"))
			no-interaction))
		(True (Put
			(State interaction-state (Variable "$face-id"))
			no-interaction))
	))

; --------------------------------------------------------
; Glancing at people.
(DefineLink
	(DefinedPredicate "glance and ack")
	(LambdaLink
		(Variable "$face-id")
		(SequentialAndLink
			(Evaluation (GroundedPredicate "py:glance_at_face")
				(ListLink (Variable "$face-id")))
			;; Mark it as acked, othwerwise, we'll keep glancing there,
			(Evaluation (Predicate "acked face")
				(ListLink (Variable "$face-id")))
		)))

;; Select a face at random, and glance at it.
(DefineLink
	(DefinedPredicate "glance at random face")
	(SequentialAnd
		(DefinedPredicate "Select random glance target")
		(Put
			(DefinedPredicate "glance and ack")
			(Get (State glance-state (Variable "$face-id")))
		)
	))

;; Glance at one of the newly-arrived faces.
;; If more than one new arrival, pick one randomly.
(DefineLink
	(DefinedSchema "glance at new person")
	(Put
		(DefinedPredicate "glance and ack")
		(RandomChoice (DefinedSchema "New arrivals"))
	))

;; Glance at the last known location of a face that is no longer
;; visible.
(DefineLink
	(DefinedSchema "glance at lost face")
	(Put
		(Evaluation (GroundedPredicateNode "py:glance_at_face")
			(ListLink (Variable "$face-id")))
		(DefinedSchema "New departures")))

; ------------------------------------------------------

; Set the interaction target
(DefineLink
	(DefinedPredicate "Set interaction target")
	(LambdaLink
		(Variable "$face-id")
		(SequentialAnd

			; Set the eye-contact state.
			(True (StateLink eye-contact-state (VariableNode "$face-id")))

			; Set the interaction state too...
			(True (StateLink interaction-state (VariableNode "$face-id")))

			; Record a timestamp
			(True (DefinedSchema "set interaction timestamp"))
		)))

;; Change the eye-contact target to a face picked randomly from the
;; crowd. (Caution: this might randomly pick the existing face...)
;;
;; This only sets the eye-contact state variable; this does NOT
;; actually cause the robot to look at them.  Use the schema
;; (DefinedSchema "look at person") to make it look.
(DefineLink
	(DefinedPredicate "Change interaction")
	(SequentialAnd
		; Pick a face at random...
		(True (Put
			(DefinedPredicate "Set interaction target")
			(DefinedSchema "Select random face")))

		; Diagnostic print
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Start new interaction")))
	))

;; Start interacting with a newly-visible face.
;;
;; This only sets the eye-contact state variable; this does NOT
;; actually cause the robot to look at them.  Use the schema
;; (DefinedSchema "look at person") to make it look.
(DefineLink
	(DefinedPredicate "interact with new person")
	; XXX Double-check that the "New arrivals" list is non-empty;
	; some OpenPsi bug sometimes sends us here, and the RandomChoice
	; crashes if the list is empty.
	(SequentialAnd
		(DefinedPredicateNode "Did someone arrive?")
		(True (Put (DefinedPredicate "Set interaction target")
			; If more than one new arrival, pick one randomly.
			(RandomChoice (DefinedSchema "New arrivals")))))
)

;; Set eye-contact face to the requested face.
;;
;; This only sets the eye-contact state variable; this does NOT
;; actually cause the robot to look at them.  Use the schema
;; (DefinedSchema "look at person") to make it look.
(DefineLink
	(DefinedPredicate "interact with requested person")
	(SequentialAnd
		(True (Put (DefinedPredicate "Set interaction target")
			(Get (State request-eye-contact-state (Variable "$x")))))
		; Now, clear the request.
		(True (Put (State request-eye-contact-state (Variable "$face-id"))
			no-interaction))
	))

;; ------------------------------------------------------------------
*unspecified*  ; Make the load be silent
