import { createTheme, MantineColorsTuple, virtualColor } from '@mantine/core';

// Define custom color tuples for the palette
const brandPrimary: MantineColorsTuple = [
  '#e6f3f9',
  '#cce7f2',
  '#99cfea',
  '#66b7e1',
  '#339fd9',
  '#0087d0', // Main brand color at index 5
  '#0070ad',
  '#00598a',
  '#004267',
  '#002b44'
];

const accent: MantineColorsTuple = [
  '#fffee6',
  '#fffccc',
  '#fff999',
  '#fff666',
  '#fff333',
  '#ddff55', // Main accent color at index 5
  '#c4e644',
  '#aacc33',
  '#91b322',
  '#779911'
];

// Custom dark colors based on the spec
const darkBackground: MantineColorsTuple = [
  '#11425d', // Secondary background (lighter)
  '#0d3448',
  '#0a2a3a',
  '#08202c',
  '#05161e',
  '#002233', // Primary background at index 5
  '#001a26',
  '#001219',
  '#000a0d',
  '#000000'
];

const lightText: MantineColorsTuple = [
  '#ffffff',
  '#f6f2e8', // Heading text
  '#e8e3d9',
  '#dad4ca',
  '#ccc5bb',
  '#c5c0c9', // Regular text at index 5
  '#b1acb5',
  '#9d98a1',
  '#89848d',
  '#757079'
];

export const theme = createTheme({
  autoContrast: true,
  colors: {
    brand: brandPrimary,
    accent: accent,
    dark: darkBackground,
    text: lightText,
    // Virtual color for dynamic accent selection
    primary: virtualColor({
      name: 'primary',
      dark: 'brand',
      light: 'brand',
    }),
  },
  primaryColor: 'accent',
  primaryShade: 5,

  // Global token overrides for dark theme
  black: '#002233', // Used as background in dark mode
  white: '#f6f2e8', // Used as text in dark mode

  // Component-specific styles
  components: {
    Button: {
      defaultProps: {
        color: 'accent',
      },
    },
    Paper: {
      styles: () => ({
        root: {
          backgroundColor: '#11425d',
        },
      }),
    },
    Card: {
      styles: () => ({
        root: {
          backgroundColor: '#11425d',
          borderColor: '#1a5270',
        },
      }),
    },
    Input: {
      styles: () => ({
        input: {
          backgroundColor: '#0d3448',
          borderColor: '#1a5270',
          color: '#c5c0c9',
          '&::placeholder': {
            color: '#757079',
          },
          '&:focus': {
            borderColor: '#ddff55',
          },
        },
      }),
    },
    TextInput: {
      styles: () => ({
        input: {
          backgroundColor: '#0d3448',
          borderColor: '#1a5270',
          color: '#c5c0c9',
          '&::placeholder': {
            color: '#757079',
          },
          '&:focus': {
            borderColor: '#ddff55',
          },
        },
      }),
    },
    Select: {
      styles: () => ({
        input: {
          backgroundColor: '#0d3448',
          borderColor: '#1a5270',
          color: '#c5c0c9',
        },
        dropdown: {
          backgroundColor: '#11425d',
          borderColor: '#1a5270',
        },
      }),
    },
    Modal: {
      styles: () => ({
        content: {
          backgroundColor: '#11425d',
        },
        header: {
          backgroundColor: '#11425d',
        },
      }),
    },
    Table: {
      styles: () => ({
        root: {
          backgroundColor: '#11425d',
        },
        thead: {
          backgroundColor: '#0d3448',
        },
      }),
    },
    // Alert color for warnings
    Alert: {
      styles: (theme: any, { color }: { color: string }) => ({
        root: {
          backgroundColor: color === 'red' ? '#f04848' : undefined,
        },
      }),
    },
  },

  // Font configuration
  fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif',
  
  // Heading styles
  headings: {
    fontWeight: '600',
    sizes: {
      h1: { fontSize: '2.25rem', lineHeight: '2.75rem' },
      h2: { fontSize: '1.875rem', lineHeight: '2.25rem' },
      h3: { fontSize: '1.5rem', lineHeight: '2rem' },
      h4: { fontSize: '1.25rem', lineHeight: '1.75rem' },
      h5: { fontSize: '1.125rem', lineHeight: '1.5rem' },
      h6: { fontSize: '1rem', lineHeight: '1.5rem' },
    },
  },
});