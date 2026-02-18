interface GamepadDebugProps {
  gamepads: Record<string, Gamepad>;
}

export function GamepadDebug({ gamepads }: GamepadDebugProps): JSX.Element {
  const gamepadDisplay = Object.keys(gamepads).map((gamepadId) => {
    const gamepad = gamepads[gamepadId];
    return (
      <div key={gamepadId}>
        <h2>{gamepad.id}</h2>
        {gamepad.buttons?.map((button, index) => (
          <div key={`btn-${index}`}>
            {index}: {button.pressed ? "True" : "False"}
          </div>
        ))}
        {gamepad.axes?.map((axis, index) => (
          <div key={`axis-${index}`}>
            {index}: {axis}
          </div>
        ))}
      </div>
    );
  });

  return <div>{gamepadDisplay}</div>;
}
