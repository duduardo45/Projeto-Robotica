import { createSystem, defaultConfig, defineConfig } from "@chakra-ui/react"

// 1. Define the "Look" centrally
const industrialConfig = defineConfig({
  theme: {
    // LAYERS: Combinations of border, background, shadow, padding
    layerStyles: {
      // The main container for panels
      panel: {
        value: {
          bg: "bg.panel",
          borderWidth: "1px",
          borderColor: "border.subtle",
          borderRadius: "sm",
          p: 5,
          boxShadow: "xs",
        },
      },
      // The high-tech, low-contrast button style
      ctrlBtn: {
        value: {
          variant: "outline",
          borderColor: "border.muted",
          color: "fg.muted",
          borderRadius: "sm",
          transition: "all 0.2s",
          _hover: {
            bg: "bg.subtle",
            color: "fg.default",
            borderColor: "fg.default",
          },
          _active: {
            bg: "bg.muted",
          },
        },
      },
      // The readout display (black background)
      readout: {
        value: {
          bg: "black",
          borderWidth: "1px",
          borderColor: "border.muted",
          borderRadius: "xs", // Sharper corners for data
          fontFamily: "mono",
        },
      },
    },
    // TEXT: Combinations of font, weight, spacing, case
    textStyles: {
      // For headers (Target X, Heading, etc.)
      label: {
        value: {
          fontSize: "xs",
          fontWeight: "medium",
          letterSpacing: "0.05em",
          textTransform: "uppercase",
          color: "fg.muted",
        },
      },
      // For data values
      monoVal: {
        value: {
          fontFamily: "mono",
          fontSize: "sm",
          fontWeight: "medium",
          color: "fg.default",
        },
      },
    },
  },
})

export const system = createSystem(defaultConfig, industrialConfig)