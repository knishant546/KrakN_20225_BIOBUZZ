# BioBuzz TeamCode

Robot application code lives here.

- `TeleopAppl`: BioBuzzTeleop and disabled driver test variants.
- `AutoAppl`: RED/BLUE Flower, Garden, Garden_NoFlwr and parking-only programs.
- `TestAppl`: diagnostic OpModes, including DisplayCooridinate.
- `subsystems`: reusable motor, servo, vision and drive components; `testClasses` holds older components.
- `config`: Auto and TeleOp tuning constants and their documentation.
- `Logging`: shared RunTelemetryLogger CSV capture.

Read [controls and tuning](README_CONTROLS_AND_TUNING.md) and [configuration reference](config/README.md) before tuning. Gate positions are placeholders requiring physical calibration. Rebuild and install after changing Java constants.