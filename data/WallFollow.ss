#define front_min 0.1f
#define side_min 0.1f
#define encoder_min 0.1f
#define normal_speed 0.5f

#var float delta_encoder_left = 0
#var float delta_encoder_right = 0
#var float ir_front = 0
#var float ir_back = 0
#var float ir_left_front = 0
#var float ir_left_back = 0
#var float ir_right_front = 0
#var float ir_right_back = 0

#param behaviour<WallFollow>& b

selector WallFollow:
    sequence Stuck:
        cond delta_encoder_left < #encoder_min
        cond delta_encoder_right < #encoder_min
        call b.publish(action::backwards_distance, 0.1f, true)
    sequence FrontWall:
        cond ir_front < #front_min
        selector Turn:
            sequence TurnLeft:
                cond ir_right_front + ir_right_back < ir_left_front + ir_left_back
                call b.publish(action::turn_left, 90.0f)
            sequence TurnRight:
                cond ir_left_front + ir_left_back < ir_right_front + ir_right_back
                call b.publish(action::turn_right, 90.0f)
    call b.publish(action::forwards_speed, #normal_speed)
