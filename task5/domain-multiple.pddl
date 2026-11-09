(define (domain MultiRobotRunner)
    (:requirements
        :strips
        :typing
        :negative-preconditions
    )

    (:types
        robot gridsquare direction counter
    )

    (:predicates
        ; Current state of each robot
        (robot-at ?r - robot ?loc - gridsquare)
        (robot-rotated ?r - robot ?d - direction)

        ; Static map information
        (gridsquare-connected ?from ?to - gridsquare ?d - direction)
        (rotation-adjacent ?from ?to - direction)

        ; A round first chooses one action for every robot, then applies them
        (choosing)
        (executing)
        (unplanned ?r - robot)

        ; Actions selected for the current round
        (chosen-move ?r - robot ?from ?to - gridsquare)
        (chosen-rotate ?r - robot ?loc - gridsquare ?from ?to - direction)
        (chosen-wait ?r - robot ?loc - gridsquare)

        ; Reservations used to reject collisions in the current round
        (destination-reserved ?loc - gridsquare)
        (edge-reserved ?from ?to - gridsquare)

        ; A small counter records how many robots have chosen/applied an action
        (remaining ?n - counter)
        (next-count ?smaller ?larger - counter)
        (zero ?n - counter)
        (robot-count ?n - counter)
    )

    ; Select a forward move for one robot.
    ; The destination cannot be selected by another robot, and the reverse
    ; edge cannot already be selected in this round.
    (:action moveforward
        :parameters (
            ?r - robot
            ?from ?to - gridsquare
            ?d - direction
            ?smaller ?larger - counter
        )
        :precondition (and
            (choosing)
            (unplanned ?r)
            (robot-at ?r ?from)
            (robot-rotated ?r ?d)
            (gridsquare-connected ?from ?to ?d)
            (remaining ?larger)
            (next-count ?smaller ?larger)
            (not (destination-reserved ?to))
            (not (edge-reserved ?to ?from))
        )
        :effect (and
            (not (unplanned ?r))
            (chosen-move ?r ?from ?to)
            (destination-reserved ?to)
            (edge-reserved ?from ?to)
            (not (remaining ?larger))
            (remaining ?smaller)
        )
    )

    ; Select a 90 degree rotation. The problem file should make
    ; rotation-adjacent true for both clockwise and anti-clockwise neighbours.
    (:action rotate
        :parameters (
            ?r - robot
            ?loc - gridsquare
            ?from ?to - direction
            ?smaller ?larger - counter
        )
        :precondition (and
            (choosing)
            (unplanned ?r)
            (robot-at ?r ?loc)
            (robot-rotated ?r ?from)
            (rotation-adjacent ?from ?to)
            (remaining ?larger)
            (next-count ?smaller ?larger)
            (not (destination-reserved ?loc))
        )
        :effect (and
            (not (unplanned ?r))
            (chosen-rotate ?r ?loc ?from ?to)
            (destination-reserved ?loc)
            (not (remaining ?larger))
            (remaining ?smaller)
        )
    )

    ; Select the do-nothing action for one robot.
    (:action wait
        :parameters (
            ?r - robot
            ?loc - gridsquare
            ?smaller ?larger - counter
        )
        :precondition (and
            (choosing)
            (unplanned ?r)
            (robot-at ?r ?loc)
            (remaining ?larger)
            (next-count ?smaller ?larger)
            (not (destination-reserved ?loc))
        )
        :effect (and
            (not (unplanned ?r))
            (chosen-wait ?r ?loc)
            (destination-reserved ?loc)
            (not (remaining ?larger))
            (remaining ?smaller)
        )
    )

    ; Every robot has now selected exactly one action.
    (:action begin-execution
        :parameters (?z - counter)
        :precondition (and
            (choosing)
            (remaining ?z)
            (zero ?z)
        )
        :effect (and
            (not (choosing))
            (executing)
        )
    )

    ; Apply a selected forward move. Occupancy of the destination is not
    ; checked here because all moves in the round are simultaneous. This is
    ; what permits following and cycle movements.
    (:action apply-moveforward
        :parameters (
            ?r - robot
            ?from ?to - gridsquare
            ?smaller ?larger - counter
        )
        :precondition (and
            (executing)
            (chosen-move ?r ?from ?to)
            (robot-at ?r ?from)
            (remaining ?smaller)
            (next-count ?smaller ?larger)
        )
        :effect (and
            (not (chosen-move ?r ?from ?to))
            (not (robot-at ?r ?from))
            (robot-at ?r ?to)
            (not (destination-reserved ?to))
            (not (edge-reserved ?from ?to))
            (unplanned ?r)
            (not (remaining ?smaller))
            (remaining ?larger)
        )
    )

    ; Apply a selected rotation.
    (:action apply-rotate
        :parameters (
            ?r - robot
            ?loc - gridsquare
            ?from ?to - direction
            ?smaller ?larger - counter
        )
        :precondition (and
            (executing)
            (chosen-rotate ?r ?loc ?from ?to)
            (robot-at ?r ?loc)
            (robot-rotated ?r ?from)
            (remaining ?smaller)
            (next-count ?smaller ?larger)
        )
        :effect (and
            (not (chosen-rotate ?r ?loc ?from ?to))
            (not (robot-rotated ?r ?from))
            (robot-rotated ?r ?to)
            (not (destination-reserved ?loc))
            (unplanned ?r)
            (not (remaining ?smaller))
            (remaining ?larger)
        )
    )

    ; Apply a selected do-nothing action.
    (:action apply-wait
        :parameters (
            ?r - robot
            ?loc - gridsquare
            ?smaller ?larger - counter
        )
        :precondition (and
            (executing)
            (chosen-wait ?r ?loc)
            (robot-at ?r ?loc)
            (remaining ?smaller)
            (next-count ?smaller ?larger)
        )
        :effect (and
            (not (chosen-wait ?r ?loc))
            (not (destination-reserved ?loc))
            (unplanned ?r)
            (not (remaining ?smaller))
            (remaining ?larger)
        )
    )

    ; All selected actions have been applied. Start the next round.
    (:action end-round
        :parameters (?n - counter)
        :precondition (and
            (executing)
            (remaining ?n)
            (robot-count ?n)
        )
        :effect (and
            (not (executing))
            (choosing)
        )
    )
)
