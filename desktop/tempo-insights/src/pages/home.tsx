import { Container, Title, Text } from '@mantine/core';
import { AppLayout } from '../components/AppLayout';

export default function HomePage() {
  return (
    <AppLayout>
      <Container>
        <Title order={2}>Home</Title>
        <Text>Center content area</Text>
      </Container>
    </AppLayout>
  );
}
