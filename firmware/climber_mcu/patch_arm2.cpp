    if (digitalRead(PIN_ACT_LIMIT_IN) == LOW) {    
        roboclaw.ResetEncoders(ROBOCLAW_ADDR);
        start_ms = millis();
        roboclaw.ForwardM1(ROBOCLAW_ADDR, 127); // Full power to break away from switch!
        while (digitalRead(PIN_ACT_LIMIT_IN) == LOW && (millis() - start_ms) < 3000) {
            delay(2);
        }
        
        for (int i = 0; i < 5; i++) {
            roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
            delay(10);
        }

        uint8_t status;
        bool valid = false;
        int32_t enc = 0;
        
        for (int i = 0; i < 5; i++) {
            enc = roboclaw.ReadEncM1(ROBOCLAW_ADDR, &status, &valid);
            if (valid) break;
            delay(20);
        }
        
        actuator_home_ticks = valid ? enc : 0;
        actuator_ready = true;
    }
