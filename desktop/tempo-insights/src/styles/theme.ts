import { createTheme, MantineColorsTuple } from '@mantine/core';

// Define custom color tuples for the palette
const brandPrimary: MantineColorsTuple = [
  '#e6f3f9',
  '#cce7f2',
  '#99cfea',
  '#66b7e1',
  '#339fd9',
  '#0087d0', // Main brand color at index 6
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
  '#ddff55', // Main accent color at index 6
  '#c4e644',
  '#aacc33',
  '#91b322',
  '#779911'
];

export const theme = createTheme({
  colorScheme: 'dark',
  colors: {
    brand: brandPrimary,
    accent: accent,
  },
  primaryColor: 'brand',
  primaryShade: 6,

  // Override default dark theme colors
  components: {
    MantineProvider: {
      styles: {
        root: {
          backgroundColor: '#002233',
          color: '#ddffee',
          minHeight: '100vh',
        }
      }
    }
  },

  globalStyles: (theme:any) => ({
    body: {
      backgroundColor: '#002233',
      color: '#ddffee',
      margin: 0,
      padding: 0,
    },
  }),
});
