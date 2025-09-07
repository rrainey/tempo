import '@mantine/core/styles.css';
import '@mantine/notifications/styles.css';
import type { AppProps } from 'next/app';
import { MantineProvider } from '@mantine/core';
import { Notifications } from '@mantine/notifications';
import { theme } from '../styles/theme';

export default function App({ Component, pageProps }: AppProps) {
  return (
    <MantineProvider theme={theme} defaultColorScheme="dark">
      <Notifications />
      <Component {...pageProps} />
    </MantineProvider>
  );
}
