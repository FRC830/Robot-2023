#include "infrastructure/PWMAngleSensor.h"

#include "infrastructure/ShuffleboardWidgets.h"

AngleSensor::AngleSensor(const int channel, const int alignment) noexcept : alignment_{alignment}
{
    // 830: REPLACED
    encoder_ = std::make_unique<frc::AnalogEncoder>(channel);
}

void AngleSensor::Periodic() noexcept
{
    if (!shuffleboard_)
    {
        return;
    }

    units::angle::degree_t commandedPosition{0.0_deg};
    units::angle::degree_t encoderPosition{0.0_deg};

    // If possible, obtain commanded and encoder positions.
    if (getCommandedAndEncoderPositionsF_)
    {
        const auto pair = getCommandedAndEncoderPositionsF_();

        commandedPosition = std::get<0>(pair);
        encoderPosition = std::get<1>(pair);
    }

    const double output = encoder_->GetAbsolutePosition();  // 830: updated for AnalogEncoder
    const auto position = GetAbsolutePosition(output, false);
    const bool status = position.has_value();
    int reportPosition{0};

    if (status)
    {
        reportPosition = position.value() + alignment_;

        if (reportPosition > 2047)
        {
            reportPosition = reportPosition - 4096;
        }
        if (reportPosition < -2048)
        {
            reportPosition = reportPosition + 4096;
        }
    }

    const double heading = static_cast<double>(reportPosition) * 360.0 / 4096.0;

    headingGyro_.Set(heading);

    double commandedError = heading - commandedPosition.to<double>();

    if (commandedError < -180.0)
    {
        commandedError += 360.0;
    }
    else if (commandedError >= 180.0)
    {
        commandedError -= 360.0;
    }

    double encoderError = heading - encoderPosition.to<double>();

    if (encoderError < -180.0)
    {
        encoderError += 360.0;
    }
    else if (encoderError >= 180.0)
    {
        encoderError -= 360.0;
    }

    // 830: removed frequency
    commandDiscrepancyUI_->GetEntry()->SetDouble(commandedError);
    statusUI_->GetEntry()->SetBoolean(status);
    commandedUI_->GetEntry()->SetDouble(commandedPosition.to<double>());
    positionUI_->GetEntry()->SetDouble(static_cast<double>(reportPosition));
    outputUI_->GetEntry()->SetDouble(output);
    encoderDiscrepancyUI_->GetEntry()->SetDouble(encoderError);
    alignmentUI_->GetEntry()->SetDouble(static_cast<double>(alignment_));
}

void AngleSensor::ShuffleboardCreate(frc::ShuffleboardContainer &container,
                                     std::function<std::pair<units::angle::degree_t, units::angle::degree_t>()> getCommandedAndEncoderPositionsF) noexcept
{
    frequencyUI_ = &container.Add("Frequency", 0)
                        .WithPosition(0, 0);

    commandDiscrepancyUI_ = &container.Add("PID Error", 0)
                                 .WithPosition(0, 1);

    statusUI_ = &container.Add("Status", false)
                     .WithPosition(0, 2)
                     .WithWidget(frc::BuiltInWidgets::kBooleanBox);

    headingUI_ = &container.Add("Heading", headingGyro_)
                      .WithPosition(1, 0)
                      .WithWidget(frc::BuiltInWidgets::kGyro)
                      .WithProperties(wpi::StringMap<nt::Value>{
                          std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});

    commandedUI_ = &container.Add("Commanded", 0)
                        .WithPosition(1, 1);

    positionUI_ = &container.Add("Position", 0)
                       .WithPosition(1, 2);

    outputUI_ = &container.Add("Output", 0.0)
                     .WithPosition(2, 0);

    encoderDiscrepancyUI_ = &container.Add("Motor Discrepancy", 0)
                                 .WithPosition(2, 1);

    alignmentUI_ = &container.Add("Alignment", 0)
                        .WithPosition(2, 2);

    getCommandedAndEncoderPositionsF_ = getCommandedAndEncoderPositionsF;

    shuffleboard_ = true;
}

std::optional<units::angle::degree_t> AngleSensor::GetAbsolutePosition() noexcept
{
    const auto position = GetAbsolutePosition(encoder_->GetAbsolutePosition(), true); // 830: switched to AnalogEncoder

    if (!position.has_value())
    {
        return std::nullopt;
    }

    return units::angle::turn_t{static_cast<double>(position.value()) / 4096.0};
}

std::optional<int> AngleSensor::GetAbsolutePositionWithoutAlignment() noexcept
{
    return GetAbsolutePosition(encoder_->GetAbsolutePosition(), false); // 830: switched to AnalogEncoder
}

// Calulate absolute turning position in the range [-2048, +2048), from the raw
// absolute PWM encoder data.  No value is returned in the event the absolute
// position sensor itself is not returning valid data.  The alignment offset is
// optionally used to establish the zero position.

// This is a low-level routine, meant to be used only by the version of
// GetAbsolutePosition() with no arguments (above) or, from test mode.  It does
// not use standardized units and leaks knowledge of the sensor output range.

// 830: removed frequency arg
std::optional<int> AngleSensor::GetAbsolutePosition(const double output, const bool applyOffset) noexcept
{
    // SRX MAG ENCODER chip seems to be AS5045B; from the datasheet, position
    // is given by:
    //   ((t_on * 4098) / (t_on + t_off)) - 1;
    //   if this result is 4096, set it to 4095.
    // This computation results in a position in the range [0, 4096).

    // 830: removed frequency check

    // Multiply by 4098 and subtract 1; should yield 0 - 4094, and also 4096.
    // Conditionals cover special case of 4096 and enforce the specified range.
    int position = std::lround(output * 4098.0) - 1;
    if (position < 0)
    {
        position = 0;
    }
    else if (position > 4095)
    {
        position = 4095;
    }

    // Now, shift the range from [0, 4096) to [-2048, +2048).
    position -= 2048;

    if (!applyOffset)
    {
        return position;
    }

    // There is a mechanical offset reflecting the alignment of the magnet with
    // repect to the axis of rotation of the wheel (and also any variance in
    // the alignment of the sensor).  Add this in here to reference position to
    // the desired zero position.
    position += alignment_;
    if (position > 2047)
    {
        position -= 4096;
    }
    if (position < -2048)
    {
        position += 4096;
    }

    return position;
}
