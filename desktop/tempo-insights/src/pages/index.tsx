import { Button, Container, Title, Text, Group } from '@mantine/core';
import { notifications } from '@mantine/notifications';

export default function Home() {
  const showNotification = () => {
    notifications.show({
      title: 'Test Notification',
      message: 'This is a demo toast notification!',
      color: 'accent',
    });
  };

  return (
    <Container>
      <Title order={1} c="accent.6">Tempo Insights</Title>
      <Text>This text should be #ddffee on a #002233 background</Text>
      <Group mt="md">
        <Button>Primary Button</Button>
        <Button color="accent">Accent Button</Button>
        <Button onClick={showNotification} variant="outline">
          Show Toast
        </Button>
      </Group>
    </Container>
  );
}
