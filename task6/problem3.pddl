; Test 3: two robots approach the same central square.
; A valid plan must prevent both robots from entering square2 in one round.
(define (problem p3-network)
    (:domain MultiRobotRunner)
    (:objects
        robot1 robot2 - robot
        square1 square2 square3 square4 square5 - gridsquare
        north east south west - direction
        c0 c1 c2 - counter
    )

    (:init
        ; Robot locations and rotations
        (robot-at robot1 square1)
        (robot-rotated robot1 east)
        (robot-at robot2 square4)
        (robot-rotated robot2 south)

        ; Cross-shaped map with square2 at the centre
        (gridsquare-connected square1 square2 east)
        (gridsquare-connected square2 square1 west)
        (gridsquare-connected square2 square3 east)
        (gridsquare-connected square3 square2 west)
        (gridsquare-connected square4 square2 south)
        (gridsquare-connected square2 square4 north)
        (gridsquare-connected square2 square5 south)
        (gridsquare-connected square5 square2 north)

        ; Clockwise and anti-clockwise 90-degree rotations
        (rotation-adjacent north east)
        (rotation-adjacent east south)
        (rotation-adjacent south west)
        (rotation-adjacent west north)
        (rotation-adjacent east north)
        (rotation-adjacent south east)
        (rotation-adjacent west south)
        (rotation-adjacent north west)

        ; Initial round state for two robots
        (choosing)
        (unplanned robot1)
        (unplanned robot2)
        (remaining c2)
        (robot-count c2)
        (next-count c0 c1)
        (next-count c1 c2)
        (zero c0)
    )

    (:goal (and
        (robot-at robot1 square3)
        (robot-at robot2 square5)
        (choosing)
    ))
)
