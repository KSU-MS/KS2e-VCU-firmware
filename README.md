# design 
balls
the vcu CAN out is still TODO mostly, currently it spits out all RX / TX traffic

below is a loose flow of the data and what handles what


## building
to build, run ```pio run -e teensy41```
## Running tests

to build and run native tests on windows, follow these instructions: https://code.visualstudio.com/docs/cpp/config-mingw 
then, run tests by calling ```pio test -e test_env```


```mermaid
    graph LR;
        state_machine --> main_loop;
        main_loop --> mcu_state;

        pedal_handler --> state_machine;
        accumulator_handler --> state_machine;
        inverter --> state_machine;

        state_machine --> pedal_handler;
        state_machine --> accumulator_handler;
        state_machine --> inverter;
        state_machine --> dash;

        inverter --> VCU_CAN_OUT;
        accumulator_handler --> VCU_CAN_OUT;
        pedal_handler --> VCU_CAN_OUT;
        state_machine --> VCU_CAN_OUT;

       

```
This is tha basic of the state machien. 
    the car is set to state


```mermaid
flowchart TD
        A[Initalize State Machine] -->|Set State to startup| B(Current state)
        B --> C(Check If Current state == New state) -->|NEW = OLD| B(Current state)
        C --> |New *does not equal* OLD| D(run exit logic)
        D --> E(run exit logic)
        E --> F(run exit logic)
        F --> B(Current State) 
```        

```mermaid
flowchart TD
    A[State set to TRACTIVE_SYSTEM_NOT_ACTIVE] -->|Enter Entry logic| B(TRACTIVE_SYSTEM_NOT_ACTIVE)
    
        B --> C(Force MC Discharge)
        C --> D(Set Dash LED Yellow)
        D --> E(Dash LED Wipe)
        E --> 

    B --> |Break to Handle| F(Handle State TRACTIVE_SYSTEM_NOT_ACTIVE)
        F --> G(Get max torque)
        G --> H(Get max RPM)
        H --> I(inverter kick 0)
        I --> J(check if precharge is attempted)
        J --> |Failure| K(set ACC_Ready to false)
        J --> |sucuess| L(set ACC_Ready to True)

        L(set ACC_Ready to True) --> M(check  if TS_active & Acc_ready)
    M --> |sucuess| N(set state to TRACTIVE_SYSTEM_ACTIVE)

```  

```mermaid

flowchart TD
    A[Initalize] -->|start| B(check if PID_MODE is active)
    B -->|Yes| C{IDGAF}
    B -->|No| D(Check If PID_TC_MODE)
    D -->|Yes| E{ALSO IDGAF}
    D --> F(SET TORQUE TO ZERO)
    F --> G(SET TORQUES)

    G --> H(SET TORQUE 1)
    G --> I(SET TORQUE 2)

    G --> J(SET ACCLERATION PERCENTS)

    J --> K(SET TORQUE 1)
    J --> L(SET TORQUE 2)

    J --> M(SET TO PERCENTAGE*)
    M --> N(IS EXPO CURVE ON?)


    N -->|Yes| O{IDGAF}
    N -->|No| P(ENTER TORQUE CHECK)

    P --> Q(RETURN CALCULATED TORQUE)

```  

## launch control state diagram

```mermaid

stateDiagram-v2

    rtd : READY_TO_DRIVE
    [*]-->rtd
    rtd --> checkButtons
    checkButtons-->toggle_lc:toggle lc if specific buttons are held
    state toggle_lc{
        toggle_get_lc_active-->set_lc_active(1):set to 1 if it was 0
        toggle_get_lc_active-->set_lc_active(0):set to 0 if it was 1
        set_lc_active(1)-->[*]
        set_lc_active(0)-->[*]
    }
    toggle_lc -->get_lc_active
    checkButtons-->get_lc_active:if buttons are not held go to check lc state
    get_lc_active-->launchState:mcu_status.get_launch_ctrl_active returns 1
    get_lc_active-->command_torque:mcu_status.get_launch_ctrl_active returns 0
    state launchState{
        idle-->waiting:gas pressed, button pressed, and no implaus
        idle-->[*]
        waiting-->idle: gas released or implaus
        waiting-->[*]
        waiting-->launching: gas still pressed and button released 
        launching-->finished: brake pressed (cancel early)
        launching-->finished: time completed
        launching-->[*]:launching in progress
        finished-->[*]:launch control done, set launch_ctrl_active to 0
    }
    launchState-->command_torque:deactivate LC when finished
    command_torque-->rtd:return to start of RTD loop
```
