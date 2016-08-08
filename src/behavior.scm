;
; behvaior.scm
;
; Configurable robot behavior tree, implemented in Atomese.
;
; Defines a set of behaviors that express Eva's personality. The
; currently-defined behaviors include acknowledging new people who enter
; the room, rotating attention between multiple occupants of the room,
; falling asleep when bored (i.e. the room is empty), and acting
; surprised when someone leaves unexpectedly.
;
; HOWTO:
; ------
; Run the main loop:
;    (run)
; Pause the main loop:
;    (halt)
;
; TODO:
; -----
; The current OpenPsi framework allows much more general and flexible
; rules than what are presented below; this geneality should be made
; use of.
;
; A general OpenPsi rule has the form of if(context) then take(action);
; these can contain variables, adn can also be classed into different
; groups based on the demands that they are fulfilling.
;
; The content below consits entirely of actions to nbe taken; the
; contexts are in the `self-model.scm` file.  The structure of the
; conexts is fairly rigid; these could probably be loosened to a
; large degree.
;
; The OpenPsi engine could be (should be?) updated to perform fuzzy
; matching on the contexts, to find close or similar contexts, if no
; one exact match can be made.  If nothing close is found, then either
; the concept blending code, or a hack of the MOSES knob-turning and
; genetic cross-over code should be used to create new quasi-random
; performance sequences from a bag of likely matches.
;
; Unit testing:
; -------------
; The various predicates below can be manually unit tested by manually
; adding and removing new visible faces, and then manually invoking the
; various rules. See faces.scm for utilities.
;
; Basic examples:
;    Manually insert a face: (make-new-face id)
;    Remove a face: (remove-face id)
;    Etc.: (show-room-state) (show-eye-contact-state) (show-visible-faces)
;
; Manually unit test the new-arrival sequence.  You can do this without
; an attached camera; she won't track the face location, but should respond.
;    (make-new-face "42")
;    (cog-evaluate! (DefinedPredicateNode "New arrival sequence"))
;    (show-acked-faces)
;    (show-room-state)
;    (show-eye-contact-state)
;    (cog-evaluate! (DefinedPredicateNode "Interact with people"))
;
; Unit test the main interaction loop:
;    (run)  ; start the behavior tree running. Should print loop iteration.
;    (make-new-face "42")  ; Should start showing expressions, gestures.
;    (remove-face "42")  ; Should retur to neutral.
;    (halt) ; Pause the loop iteration; (run) will start it again.
;
; Unit test chatbot:
;    (State chat-state chat-start) ; to simulate having it talk.
;    (State chat-state chat-stop)  ; to signal that talking has stopped.
;
; --------------------------------------------------------
; Character engine notes
; The character engine is an older incarnation of this code, predating
; the owyl behavior trees.  The below are notes abouit some of the
; things it did, and where to look for equivalents here.
;
; 1) Room-state transtions
; -- no faces present for a while
;     (cog-evalute! (DefinedPredicateNode "Search for attention"))
; -- no-face just transitioned to one-face
;     (cog-evaluate! (DefinedPredicateNode "Was Empty Sequence"))
; -- no-face just transitioned to multiple-faces
; -- one-face just transitioned to no-face
; -- one-face just transitioned to multiple-faces
;      (DefinedPredicate "Respond to new arrival")
;      (DefinedPredicateNode "Interacting Sequence")
; -- one face present for a while
; -- multiple faces present for a while
; -- multiple-faces just transitioned to one-face
; -- multiple-faces just transitioned to no-face
;
; 2) Interactions depend on whether person is known or not.
;
; -- it's a familiar face
; -- it's a familiar face to whom the robot has already been introduced
; -- it's an unfamiliar face
;
; 3) Familiar-face interactions
;
; -- prior emotions when interacting with that face were positive
; -- prior emotions when interacting with that face were negative
; -- prior emotions when interacting with that face were neutral
;
; 4) Introduction sequence
;    If the robot is talking to someone with whom it has not previously
;    been introduced, it can enter into a short procedure aimed at
;    gathering the person's name...
;
; 5) Adhoc animation patterns
;
; -- changing a subject (maybe the AIML can send a signal indicating
;    when the subject is being changed?)
; -- asking a question of the user
; -- starting to talk after a period of being silent
; -- ending an interaction with one guy before shifting attention to
;    another guy
; -- starting an interaction with a new guy after shifting attention
;    from another guy
;
; --------------------------------------------------------
;
; This will be used to tag the name of the module making the request.
; Everything in this module is the behavior tree, so that is what we
; call it.
(define bhv-source (Concept "Behavior Tree"))

; ------------------------------------------------------
; More complex interaction sequences.

;; Interact with the current face target.
;; XXX Needs to be replaced by OpenPsi emotional state modelling.
(DefineLink
	(DefinedPredicate "Interact with face")
	(SequentialAnd
		;; Look at the interaction face
		(DefinedPredicate "look at person")

		;; Show random expressions only if NOT talking
		(SequentialOr
			;(Not (DefinedPredicate "chatbot is listening?"))
			(SequentialAnd

				(SequentialOrLink
					(NotLink (DefinedPredicateNode "Time to change expression"))
					(DefinedPredicateNode "Show positive expression")
				)
				(SequentialOrLink
					(NotLink (DefinedPredicateNode "Time to make gesture"))
					(DefinedPredicateNode "Pick random positive gesture"))
		))
	))

; ------------------------------------------------------
;; Was Empty Sequence - if there were no people in the room, then
;; look at the new arrival.
;;
;; (cog-evaluate! (DefinedPredicateNode "Was Empty Sequence"))
(DefineLink
	(DefinedPredicate "Was Empty Sequence")
	(SequentialAnd
		(DefinedPredicate "was room empty?")
		; Record a new emotional state (for self-awareness)
		; XXX FIXME this should be a prt of "Show random expression"
		; below ...
		(Put (DefinedPredicate "Request Set Emotion State")
			(ListLink bhv-source (Concept "new-arrival")))

		(DefinedPredicate "interact with new person")
		(DefinedPredicate "look at person")
		(Put (DefinedPredicate "Show random expression")
			(ConceptNode "new-arrival"))
		(Put (DefinedPredicate "Publish behavior")
			(Concept "Look at new arrival"))
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Look at newly arrived person")))
	))

;; If interaction is requested, then interact with that specific person.
;; Make sure we look at that person ...
(DefineLink
	(DefinedPredicate "Interaction requested action")
	(SequentialAnd
		(True (DefinedPredicate "If sleeping then wake"))
		(True (DefinedPredicate "If bored then alert"))
		(DefinedPredicate "interact with requested person")
		(DefinedPredicate "look at person")
		(Put (DefinedPredicate "Publish behavior")
			(Concept "Look at requested face"))
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Looking at requested face")))
	))

;; Sequence - Currently interacting with someone when a new person
;  becomes visible.
; (cog-evaluate! (DefinedPredicateNode "Interacting Sequence"))
(DefineLink
	(DefinedPredicate "Interacting Sequence")
	(SequentialAnd
		(DefinedPredicate "Is interacting with someone?")
		(DefinedPredicate "dice-roll: glance new face")
		(True (DefinedSchema "glance at new person"))
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Glance at new person")))
	))

;; Respond to a new face becoming visible.
;
;; XXX TODO -- if interacting for a while
;; this alters probability of glance...
(DefineLink
	(DefinedPredicate "Respond to new arrival")
	(SequentialOr
		(DefinedPredicate "Was Empty Sequence")
		(DefinedPredicate "Interacting Sequence")
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Ignoring new person")))
		(True)
	))

;; Return false if not sleeping.
;; Return true if we were sleeping, and we woke up.
(DefineLink
	(DefinedPredicate "If sleeping then wake")
	(SequentialAnd
		(DefinedPredicate "Is sleeping?")
		(DefinedPredicate "Wake up")))

;; If soma state was bored, change state to alert.
;; If we don't do this, then we risk being bored while also talking,
;; which will make us fall asleep if bored too long (narcoleptic).
;;
;; Return false if not bored.
;; Return true if we were bored, and we woke up.
(DefineLink
	(DefinedPredicate "If bored then alert")
	(SequentialAnd
		(DefinedPredicate "Is bored?")
		(Evaluation (DefinedPredicate "Request Set Soma State")
			(ListLink bhv-source soma-awake))))

;; Check to see if a new face has become visible.
(DefineLink
	(DefinedPredicate "New arrival sequence")
	(SequentialAnd
		(True (DefinedPredicate "If sleeping then wake"))
		(True (DefinedPredicate "If bored then alert"))
		(DefinedPredicate "Respond to new arrival")
		(DefinedPredicate "Update status")
	))

;; Check to see if someone left.
(DefineLink
	(DefinedPredicate "Someone left action")
	(SequentialAnd
		(Put (DefinedPredicate "Publish behavior")
			(Concept "Someone left"))
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Someone left")))
		(SequentialOr
			; Were we interacting with the person who left? If so,
			; look frustrated, return head position to neutral.
			(SequentialAnd
				(Equal
					(DefinedSchema "New departures")
					(Get (State eye-contact-state (Variable "$x"))))
				(DefinedPredicate "Show frustrated expression")
				(DefinedPredicate "return to neutral")
			)
			;; Were we interacting with someone else?  If so, then
			;; maybe glance at the location of the person who left.
			(SequentialAnd
				(DefinedPredicate "Is interacting with someone?")
				(SequentialOr
					(NotLink (DefinedPredicate "dice-roll: glance lost face"))
					(FalseLink (DefinedSchema "glance at lost face"))
					(Evaluation (GroundedPredicate "scm: print-msg")
						(ListLink (Node "--- Glance at lost face"))))
				(TrueLink)
			)
			(Evaluation (GroundedPredicate "scm: print-msg")
				(ListLink (Node "--- Ignoring lost face")))
			(TrueLink)
		)
		;; Clear the lost face target
		(DefinedPredicate "Clear lost face")
		(DefinedPredicate "Update room state")
	))

;; Collection of things to do while interacting with people.
;; Evaluates to true if there is an ongoing interaction with
;; someone.
(DefineLink
	(DefinedPredicate "Interact with people")
	(SequentialAnd
		; Say something, if no one else has said anything in a while.
		; i.e. if are being ignored, then say something.
;		(SequentialOr
;			(SequentialAnd
;				(DefinedPredicate "Silent too long")0
;				(Evaluation (GroundedPredicate "scm: print-msg")
;					(ListLink (Node "--- Everyone is ignoring me!!!")))
;				(Put (DefinedPredicate "Publish behavior")
;					(Concept "Sound of crickets"))
;				(True (DefinedSchema "set heard-something timestamp"))
;			)
;			(True)
;		)

		; This sequential-or is true if we're not interacting with anyone,
		; or if there are several people and its time to change up.
		(SequentialOr
			; Start a new interaction, if we've been interacting with
			; someone for too long.
			(SequentialAnd
				(SequentialOr
					(Not (DefinedPredicate "Is interacting with someone?"))
					(SequentialAnd
						(DefinedPredicate "More than one face visible")
						(DefinedPredicate "Time to change interaction")))
				; Select a new face target, interact with it.
				(DefinedPredicate "Change interaction")
				(DefinedPredicate "Interact with face")
				(Put (DefinedPredicate "Publish behavior")
					(Concept "Interact with someone else"))
			)

			; ##### Glance At Other Faces & Continue With The Last Interaction
			(SequentialAnd
				; Gets called 10x/second; don't print.
				;(Evaluation (GroundedPredicate "scm: print-msg")
				;	(ListLink (Node "--- Continue interaction")))
				(SequentialOr
					(SequentialAnd
						(DefinedPredicate "More than one face visible")
						(DefinedPredicate "dice-roll: group interaction")
						(DefinedPredicate "glance at random face"))
					(True))
				(DefinedPredicateNode "Interact with face")
				(SequentialOr
					(SequentialAnd
						(DefinedPredicateNode "dice-roll: face study")
; XXX incomplete!  need the face study saccade stuff...
; I am confused ... has this been superceeded by the ROS-saccades,
; or is this still means to be used?
						(False)
					)
					(True))
			)
			(DefinedPredicate "Is interacting with someone?")
		)
	))

; ------------------------------------------------------
; Empty-room behaviors. We either search for attention, or we sleep,
; or we wake up.

(DefineLink
	(DefinedPredicateNode "Search for attention")
	(SequentialAndLink
		; Proceed only if we are allowed to.
		(Put (DefinedPredicate "Request Set Emotion State")
			(ListLink bhv-source (ConceptNode "bored")))

		; If the room is empty, but we hear people talking ...
		(True (SequentialAnd
			(DefinedPredicate "Heard Something?")
			(Put (DefinedPredicate "Publish behavior")
				(Concept "Who is there?"))
			; Well, we are not bored, if we hear strange noises.
			(True (DefinedSchema "set bored timestamp"))
		))

		; Pick a bored expression, gesture
		(SequentialOr
			(Not (DefinedPredicate "Time to change expression"))

			; Tell ROS that we are looking for attention, ... but not
			; too often. Piggy-back on "Time to change expression" to
			; rate-limit the message.
			(False (Put (DefinedPredicate "Publish behavior")
				(Concept "Searching for attention")))
			(PutLink (DefinedPredicateNode "Show random expression")
				(ConceptNode "bored"))
		)
		(SequentialOr
			(Not (DefinedPredicate "Time to make gesture"))
			(PutLink (DefinedPredicateNode "Show random gesture")
				(ConceptNode "bored")))

		;; Search for attention -- change gaze every so often.
		;; Coordinate system: x forward; y side-to-side, z up.
		;; XXX question: This is turning the whole head; perhaps we
		;; should be moving eyes only?
		(SequentialOr
			(Not (DefinedPredicate "Time to change gaze"))
			(SequentialAnd
				(Evaluation (DefinedPredicate "Look at point")
					(ListLink ;; three numbers: x,y,z
						(Number 1)
						(RandomNumber
							(DefinedSchema "gaze right max")
							(DefinedSchema "gaze left max"))
						(Number 0)))
				(True (DefinedSchema "set attn-search timestamp"))
			))
	))

; Call once, to fall asleep.
(DefineLink
	(DefinedPredicate "Go to sleep")
	(SequentialAnd
		; Proceed only if we are allowed to.
		(Put (DefinedPredicate "Request Set Emotion State")
			(ListLink bhv-source (ConceptNode "sleepy")))

		; Proceed with the sleep animation only if the state
		; change was approved.
		(Evaluation (DefinedPredicate "Request Set Soma State")
			(ListLink bhv-source soma-sleeping))

		(Evaluation (GroundedPredicate "scm: print-msg-time")
			(ListLink (Node "--- Go to sleep.")
				(Minus (TimeLink) (DefinedSchema "get bored timestamp"))))
		(True (DefinedSchema "set sleep timestamp"))

		(Put (DefinedPredicate "Publish behavior")
			(Concept "Falling asleep"))

		; First, show some yawns ...
		(Put (DefinedPredicate "Show random gesture")
			(Concept "sleepy"))

		; Finally, play the go-to-sleep animation.
		(Evaluation (GroundedPredicate "py:do_go_sleep") (ListLink))
	))

; Wake-up sequence
(DefineLink
	(DefinedPredicate "Wake up")
	(SequentialAnd
		; Request change soma state to being awake. Proceed only if
		; the request is accepted.
		(Evaluation (DefinedPredicate "Request Set Soma State")
			(ListLink bhv-source soma-awake))

		; Proceed only if we are allowed to.
		(Put (DefinedPredicate "Request Set Emotion State")
			(ListLink bhv-source (ConceptNode "wake-up")))

		(Evaluation (GroundedPredicate "scm: print-msg-time")
			(ListLink (Node "--- Wake up!")
				(Minus (TimeLink) (DefinedSchema "get sleep timestamp"))))

		(Put (DefinedPredicate "Publish behavior")
			(Concept "Waking up"))

		; Reset the bored timestamp, as otherwise we'll fall asleep
		; immediately (cause we're bored).
		(True (DefinedSchema "set bored timestamp"))

		; Reset the "heard something" state and timestamp.
		(True (DefinedPredicate "Heard Something?"))
		(True (DefinedSchema "set heard-something timestamp"))

		; Run the wake animation.
		(Evaluation (GroundedPredicate "py:do_wake_up") (ListLink))

		; Also show the wake-up expression (head shake, etc.)
		(Put (DefinedPredicate "Show random expression")
			(Concept "wake-up"))
		(Put (DefinedPredicate "Show random gesture")
			(Concept "wake-up"))
	))

;; Collection of things to do if nothing is happening (i.e. if no faces
;; are visible).
;; -- Go to sleep if we've been bored for too long.
;; -- Wake up if we've slept too long, or if we heard noises (speech)
(DefineLink
	(DefinedPredicate "Nothing is happening")
	(SequentialAnd

		; If we are not bored already, and we are not sleeping,
		; and we didn't hear any noises, then we are bored now...
		(SequentialOr
			(DefinedPredicate "Is bored?")
			(DefinedPredicate "Is sleeping?")
			(SequentialAnd
				(DefinedPredicate "Heard Something?")
				(True (Put (DefinedPredicate "Publish behavior")
					(Concept "What was that sound?")))
				; We are not bored, if we are hearing strange noises.
				(True (DefinedSchema "set bored timestamp"))
			)
			(SequentialAnd
				(Evaluation (DefinedPredicate "Request Set Soma State")
					(ListLink bhv-source soma-bored))

				(True (DefinedSchema "set bored timestamp"))

				(Put (DefinedPredicate "Publish behavior")
					(Concept "This is boring"))

				; ... print output.
				(Evaluation (GroundedPredicate "scm: print-msg")
					(ListLink (Node "--- Bored! nothing is happening!")))
			))

		(SequentialOr
			; If we're not sleeping yet, then fall asleep
			(SequentialAnd
				; Proceed only if not sleeping ...
				(Not (DefinedPredicate "Is sleeping?"))

				(SequentialOr
					; ##### Go To Sleep #####
					(SequentialAnd
						(DefinedPredicate "Bored too long")
						(DefinedPredicate "Go to sleep"))

					; ##### Search For Attention #####
					; If we didn't fall asleep above, then search for attention.
					(DefinedPredicate "Search for attention")
				))

			; If we are sleeping, then maybe its time to wake?
			(SequentialOr
				; Maybe its time to wake up ...
				(SequentialAnd
					(SequentialOr
						; Did we sleep for long enough?
						(DefinedPredicate "Time to wake up")
						(SequentialAnd
							(DefinedPredicate "Heard Something?")
							(True (Put (DefinedPredicate "Publish behavior")
								(Concept "What was that sound?")))
							; We are not bored, if we are hearing strange noises.
							(True (DefinedSchema "set bored timestamp"))
						)
					)
					(DefinedPredicate "Wake up")
				)
				; ##### Continue To Sleep #####
				; Currently, a no-op...
				(SequentialAndLink
					(TrueLink)
					;(Evaluation (GroundedPredicate "scm: print-msg")
					;	(ListLink (Node "--- Continue sleeping.")))
				)
			)
		)
))

;; ------------------------------------------------------------------
;; Chat-related behaviors.
;;
;; These can be unit-tested by saying
;;    rostopic pub --once chat_events std_msgs/String speechstart
;;    rostopic pub --once chat_events std_msgs/String speechend

; Things to do, if TTS vocalization just started.
(DefineLink
	; owyl "chatbot_speech_start()" method
	(DefinedPredicate "Speech started")
	(SequentialAnd
		; Switch to face-study saccade ...
		(Evaluation (GroundedPredicate "py:conversational_saccade")
				(ListLink))
		; ... and show one random gesture from "conversing" set.
		(Put (DefinedPredicate "Show random gesture")
			(ConceptNode "conversing"))
		; ... and also, sometimes, the "chatbot_positive_nod"
		(Put (DefinedPredicate "Show random gesture")
			(ConceptNode "chat-positive-nod"))
		; ... and switch state to "talking"
		(True (Put (State chat-state (Variable "$x")) chat-talk))

		; ... print output.
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Start talking")))
))

; Currently used for scripted behaviors while STT doesn't publish
; accurate events.
(DefineLink
	(DefinedPredicate "Listening started")
	(SequentialAnd
		; Switch to face-study saccade ...
		(Evaluation (GroundedPredicate "py:listening_saccade")
				(ListLink))
		; ... and show one random gesture from "conversing" set.
		(Put (DefinedPredicate "Show random gesture")
			(ConceptNode "listening"))
		; ... and also, sometimes, the "chatbot_positive_nod"
		(Put (DefinedPredicate "Show random gesture")
			(ConceptNode "chat-positive-nod"))
		; ... and switch state to "listen"
		(True (Put (State chat-state (Variable "$x")) chat-listen))

		; ... print output.
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Start Listen")))
))

;; Things to do, if the chatbot is currently talking.
(DefineLink
	(DefinedPredicate "Speech ongoing")
	(SequentialAnd
		; Handle the various affect states.
		(SequentialOr
			(SequentialAnd
				; If chatbot is happy ...
				(DefinedPredicate "chatbot is happy")
				; ... show one of the neutral-speech expressions
				(SequentialOr
					(Not (DefinedPredicate "Time to change expression"))
					(Put (DefinedPredicateNode "Show random expression")
						(ConceptNode "neutral-speech")))

				; ... nod slowly ...
				(SequentialOr
					(Not (DefinedPredicate "Time to make gesture"))
					(SequentialAnd
						(Put (DefinedPredicate "Show random gesture")
							(ConceptNode "chat-positive-nod"))

						; ... raise eyebrows ...
						(Put (DefinedPredicate "Show random gesture")
							(ConceptNode "chat-pos-think"))

						; ... switch to chat fast blink rate...
						(Evaluation (GroundedPredicate "py:blink_rate")
							(ListLink
								(DefinedSchema "blink chat fast mean")
								(DefinedSchema "blink chat fast var")))
				))
			)
			(SequentialAnd
				; If chatbot is not happy ...
				(DefinedPredicate "chatbot is negative")
				; ... show one of the frustrated expressions
				(SequentialOr
					(Not (DefinedPredicate "Time to change expression"))
					(Put (DefinedPredicateNode "Show random expression")
						(ConceptNode "frustrated")))
				(SequentialOr
					(Not (DefinedPredicate "Time to make gesture"))
					(SequentialAnd
						; ... shake head quickly ...
						(Put (DefinedPredicate "Show random gesture")
							(ConceptNode "chat-negative-shake"))
						; ... furrow brows ...
						(Put (DefinedPredicate "Show random gesture")
							(ConceptNode "chat-neg-think"))
						; ... switch to chat slow blink rate...
						(Evaluation (GroundedPredicate "py:blink_rate")
							(ListLink
								(DefinedSchema "blink chat slow mean")
								(DefinedSchema "blink chat slow var")))
				))
			))))

; Things to do, if the chattbot stopped talking.
(DefineLink
	(DefinedPredicate "Speech ended")
	(SequentialAnd
		; Switch back to exploration saccade ...
		(Evaluation (GroundedPredicate "py:explore_saccade")
			(ListLink))

		; ... switch to normal blink rate...
		(Evaluation (GroundedPredicate "py:blink_rate")
			(ListLink
				(DefinedSchema "blink normal mean")
				(DefinedSchema "blink normal var")))

		; ... and switch state to "listening"
		(True (Put (State chat-state (Variable "$x")) chat-idle))

		; ... and print some tracing output
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Finished talking")))
	))

; Things to do, if stopped listening.
(DefineLink
	(DefinedPredicate "Listening ended")
	(SequentialAnd
		; Switch back to exploration saccade ...
		(Evaluation (GroundedPredicate "py:explore_saccade")
			(ListLink))

		; ... switch to normal blink rate...
		(Evaluation (GroundedPredicate "py:blink_rate")
			(ListLink
				(DefinedSchema "blink normal mean")
				(DefinedSchema "blink normal var")))

		; ... and switch state to "idle"
		(True (Put (State chat-state (Variable "$x")) chat-idle))

		; ... and print some tracing output
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Finished talking")))
	))

; Things to do, if the chatbot is listening.
(DefineLink
	(DefinedPredicate "Listening ongoing")
	(SequentialAnd
        ; Show one of the neutral-speech expressions
        (SequentialOr
            (Not (DefinedPredicate "Time to change expression"))
            (Put (DefinedPredicateNode "Show random expression")
                (ConceptNode "neutral-listen")))

        ; ... nod slowly ...
        (SequentialOr
            (Not (DefinedPredicate "Time to make gesture"))
            (SequentialAnd
                (Put (DefinedPredicate "Show random gesture")
                    (ConceptNode "chat-positive-nod"))

                ; ... raise eyebrows ...
                (Put (DefinedPredicate "Show random gesture")
                    (ConceptNode "chat-pos-think"))
        ))
		(TrueLink)
	))

(DefineLink
	(DefinedPredicate "Keep alive")
	(SequentialAnd
		; If the chatbot stopped talking ...
        (SequentialOr
            (Not (DefinedPredicate "Time to change expression"))
            (Put (DefinedPredicateNode "Show random expression")
                (ConceptNode "neutral-listen")))

        ; ... nod slowly ...
        (SequentialOr
            (Not (DefinedPredicate "Time to make gesture"))
            (SequentialAnd
                (Put (DefinedPredicate "Show random gesture")
                    (ConceptNode "chat-positive-nod"))
                ; ... raise eyebrows ...
                (Put (DefinedPredicate "Show random gesture")
                    (ConceptNode "chat-pos-think"))
        ))
		(TrueLink)
	))

; ----------------------------------------------------------------------
;; Actions for loud sound
(DefineLink
	(DefinedPredicate "Say whoa!")
	(Put (DefinedPredicate "Say") (Node "whoa!")))

; ----------------------------------------------------------------------
