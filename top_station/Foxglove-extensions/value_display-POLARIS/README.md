# Value Display Extension

A Foxglove extension for displaying scalar values with customizable formatting, styling, and mathematical transformations.

## Features

- **Display scalar values** from ROS 2 topics
- **Customizable styling**: font size, color, background, alignment, bold, and italic
- **Mathematical transformations**: abs, ceil, floor, round, sqrt, pow2, exp, log, sin, cos, tan, 1/x
- **Precision control**: Set decimal precision for numerical display
- **Auto font sizing**: Automatically scale font size based on panel height
- **Units display**: Add custom units to displayed values

## Installation

1. Build the extension:
```bash
npm install
npm run build
```

2. Package the extension:
```bash
npm run package
```

3. Install in Foxglove Studio:
- Open Foxglove Studio
- Go to Extensions → Install extension
- Select the built `.foxe` file

## Topics

This extension automatically handles any scalar value (number) from your ROS 2 topics. Simply select a topic and field path in the settings.

## Settings

- **Data**: Select the topic and message field to display
- **Display**: Configure visual appearance (colors, fonts, alignment, units)
- **Numerical**: Set precision and mathematical transformations

## Development

```bash
# Install dependencies
npm install

# Build for development
npm run build

# Build for production
npm run foxglove:prepublish

# Lint and fix code
npm run lint

# Local installation for testing
npm run local-install
```

## License

MIT
