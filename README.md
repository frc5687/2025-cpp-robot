# Design Doc
This contains the thoughts for the C++ version of our robot code, it contains mostly the same concepts that we used in java but ported to c++

## Building:
To build code run: `./gradlew build`</b>

To Simulate without debugger: `./gradlew simulateNative`</b>

To format with wpiformat: `python3 wpiformat` (Need python and wpiformat installed)</b>

## Classes with Header and CPP files
### Headers:
A class typically consists of two files, a header file (`.h` or `.hpp`) and a source file (`.cpp`, `.cc`).
The header file is meant to contain the structure and API of the class, this would be showing what the class does, not how it does it.
This typically includes function declarations, member variables, and any public interfaces your class exposes.
```cpp
#pragma once

class Foo {
public:
    Foo();
    void CalculateBar();
    int Bar() const { return m_bar; }
private:
    int m_bar;
}
```
As shown above there are some execptions to this rule.
If an implemented requires only one line (like a getter) we write the implementation in the header file.
Another big exception is templated classes. Since templates are compiled when they’re instantiated,
the compiler needs to see the full definition in order to generate the code for each specific type.
That’s why template implementations typically live in the header file as well.</b>

__NOTE__: You can you define implementaion in a `.tpp` file which is meant for the implementation of the templated class,
but you will not see this often, [here is a decent explantation to this](https://stackoverflow.com/a/495056).

## IO Structure
We use a depenency injection (DI) methedology for all of our subsystems.
There are two componts for the "base" IO, which is the `Inputs` data type (typically a struct) and the IO class.

The `Inputs` struct is usually named `SubsystemInputsIO`, where subsystem corresponds to the system being referenced.
This struct contains the values that are important for the subsystem to be updated or measured from hardware. Examples include motor position, motor velocity, motor temperature, solenoid position, etc. In general, this means any variable that represents either:

* A direct measurement from hardware to your system, or
* A value derived from a measurement (e.g., linear position derived from motor position after gear reduction and wheel circumference).

The other component is the virtual class `SubsystemIO`. A virtual class in C++ serves the same purpose as an interface in Java. This is a contract that enforces implementation of certain methods in derived classes. In this case, the `UpdateInputs` method must be implemented in any class inheriting from `SubsystemIO`. This requirement is denoted by the `virtual` keyword and the `= 0` syntax.

```cpp
struct SubsystemInputsIO {
    units::volt_t motorVoltage = 0_V;
    units::turn_t motorPosition = 0_tr;

    units::meter_t subsystemLinearPosition = 0_m;
};
class SubsystemIO {
public:
    virtual UpdateInputs(SubsystemInputs& inputs) = 0;
    // Other methods that are required per subsystem to function (SetPostion, SetAngle, etc)
};
```

This then gets used in the derived classes, in our case you will mostly see `CTRESubsystemIO` and `SimSubsystemIO`. We typically only use CTRE devices since this was the prefered hardware (and software) the team uses. As you might expect, `CTRESubsystemIO` containts the implementation of `SubsytemIO` which CTRE hardware. This would include setting up CTRE specific API calls to our specific hardware for the subsystem. As for `SimSubsystemIO`, this is the implementation to simulate the subsystem we are creating.

```cpp
// Derived class using CTRE hardware
class CTRESubsystemIO : public SubsystemIO {
public:
    void UpdateInputs(SubsystemInputsIO& inputs) override {
        // Example functions
        inputs.motorVoltage = m_motor.GetVoltage();
        inputs.motorPosition = m_motor.GetPosition();
        inputs.subsystemLinearPosition =
            inputs.motorPosition * kGearReduction * kWheelCircumference / 1_tr; // turns * unitless * meters / turns = meters
    }

    void SetVoltage(units::volt_t voltage) {
        m_motor.SetConstrol(m_voltage.WithOutput(voltage));
    }

private:
    ctre::phoenix6::hardware::TalonFX m_motor{1};
    ctre::phoenix6::controls::VoltageOut m_voltage{0_V};
};

// Derived class for simulation
class SimSubsystemIO : public SubsystemIO {
public:
    void UpdateInputs(SubsystemInputsIO& inputs) override {
        // This is just a really simple example, we would typically use WPILib sim classes
        // For elevator, motor, arm, etc.
        simPosition += simVelocity * kDt;
        inputs.motorVoltage = 0_V;
        inputs.motorPosition = units::turn_t{simPosition};
        inputs.subsystemLinearPosition =
            units::meter_t{simPosition * kGearReduction * kWheelCircumference};
    }

    void SetSimVelocity(double velocity) {
        simVelocity = velocity;
    }
private:
    double m_simPosition = 0.0;
    double m_simVelocity = 0.0;
};
```
## Subsystems
A subsystem is the higher-level abstraction that sits on top of SubsystemIO. It represents the logic, commands, and state transitions of a system rather than the hardware-level details.

* Subsystems contain logic for game specific operations (e.g., setting elevator height to predefined scoring levels L4, L3, L2 for the 2025 FRC game).

* IO classes do not contain this game logic, they only provide raw or derived measurements and low-level controls.

This separation allows the same subsystem logic to run in hardware, simulation, or testing environments by simply swapping out the IO implementation.

For example an elevator:

```cpp

struct HeightLevel {
    std::string name; // Could be L2, L3, L4, Algae L1, Algae L2, etc.
    unit::meter_t height; // height to set the elevator of our system in meters
};

class ElevatorSubsystem {
public:
    ElevatorSubsystem(std::unique_ptr<SubsystemIO> io) : m_io(std::move(io)) {}

    void Periodic() {
        m_io->UpdateInputs(m_inputs);
    }

    void SetHeight(HeightLevel level) {
        // Maybe you want to cache this for logging (you can also just log when this is called as well)
        m_io->SetPosition(level.height);
    }

private:
    SubsystemInputsIO m_inputs;
    std::unique_ptr<SubsystemIO> m_io;
};
```


# Resources:
[WPILib docs](docs.wpilib.org) All things come back to here, this provides everything you could ever need with resepect to FRC robotics<br />
[Control Engineering in FRC](https://controls-in-frc.link/) by Tyler Veness:
This is probably #1 resource for more complex controls for what we use and a lot of deriviation stem (if not taken) are from this book
