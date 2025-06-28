while (1)
  {
    // Пример: 1000 шагов по часовой стрелке, пауза, 10000 шагов против
    printf("[MKS] Move 1000 steps CW (Position Mode 1)\r\n");
    MksServo_PositionMode1Run(&mksServo, 1, 3000, 1, 122880); // 1 - по часовой, speed=3000, acc=1, steps=122880
    MksServo_WaitForACK(&mksServo, 5, 3000);                  // ждем старт
    MksServo_WaitForACK(&mksServo, 5, 10000);                 // ждем завершение
    HAL_Delay(2000);                                          // пауза 2 секунды
    printf("[MKS] Move 10000 steps CCW (Position Mode 1)\r\n");
    MksServo_PositionMode1Run(&mksServo, 0, 3000, 1, 122880); // 0 - против часовой
    MksServo_WaitForACK(&mksServo, 5, 3000);
    MksServo_WaitForACK(&mksServo, 5, 10000);

    // После движения — запрос статуса драйвера
    if (MksServo_ReadStatus(&mksServo, 100)) {
        printf("[MKS] Locked-rotor: %d, Encoder: %u\r\n", mksServo.status.locked_rotor, mksServo.status.encoder_position);
    } else {
        printf("[MKS] Status read error\r\n");
    }

    // После движения — запрос ошибки позиции
    int32_t pos_error = 0;
    if (MksServo_ReadPositionError(&mksServo, &pos_error, 300)) {
        printf("[MKS] Position error: %ld\r\n", pos_error);
    } else {
        printf("[MKS] Position error read error\r\n");
    }

    //********************************************************* */
    PollPositionErrorTask();
    StateMachine_loop();
    // Условный выбор действий по текущему состоянию
    switch (stateMachine.getState())
    {
    case State::Initial:
      // TODO: действия для Initial
      break;
    case State::Manual:
      // TODO: действия для Manual

      break;
    case State::GiroScope:

      break;
    case State::Scan:
      // TODO: действия для Scan
      break;
    case State::BindMode:
      // TODO: действия для BindMode
      break;
    case State::Calibrate:
      // TODO: действия для Calibrate

      break;
    case State::CalibrateAndBind:
      // TODO: действия для CalibrateAndBind
      break;
    default:
      // TODO: обработка неизвестного состояния
      break;
    }
  }
  /* USER CODE END 3 */
}



//************************************** */

 if (buttonsState.turn_left == BUTTON_ON && !flag_first_run) // Если педаль нажата
      {

        flag_first_run = true;                             // Флаг для первого запуска
        MksServo_SpeedModeRun(&mksServo, 0x01, 1500, 200); //

        printf("[MKS] Servo running left\r\n");
      }
      else if (buttonsState.turn_right == BUTTON_ON && !flag_first_run) // Если педаль нажата
      {

        flag_first_run = true;                             // Флаг для первого запуска
        MksServo_SpeedModeRun(&mksServo, 0x00, 1500, 200); //

        printf("[MKS] Servo running right\r\n");
      }
      if (buttonsState.turn_right == BUTTON_OFF && buttonsState.turn_left == BUTTON_OFF && flag_first_run)
      {
        flag_first_run = false; // Сброс флага после отпускания педалей

        MksServo_SpeedModeRun(&mksServo, 0x00, 0, 0); // stop servo
                                                      // MksServo_EmergencyStop_Simple(&mksServo);
        printf("[MKS] Servo stopped\r\n");
      }
    }
