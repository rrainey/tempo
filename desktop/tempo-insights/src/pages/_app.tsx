import '@mantine/core/styles.css';
import '@mantine/notifications/styles.css';
import '../styles/globals.css';
import type { AppProps } from 'next/app';
import { MantineProvider } from '@mantine/core';
import { Notifications } from '@mantine/notifications';
import { theme } from '../styles/theme';

export default function App({ Component, pageProps }: AppProps) {
  return (
    <MantineProvider 
      theme={theme} 
      defaultColorScheme="dark"
      cssVariablesResolver={(theme) => ({
        variables: {},
        light: {},
        dark: {
          // Override CSS variables for dark mode
          '--mantine-color-body': '#002233',
          '--mantine-color-text': '#c5c0c9',
          '--mantine-color-dimmed': '#757079',
          '--mantine-color-placeholder': '#757079',
          '--mantine-color-anchor': '#ddff55',
          '--mantine-color-default-border': '#1a5270',
          '--mantine-color-default': '#11425d',
          '--mantine-color-default-hover': '#1a5270',
          '--mantine-color-default-color': '#c5c0c9',
        },
      })}
    >
      <Notifications position="top-right" />
      <Component {...pageProps} />
    </MantineProvider>
  );
}