import { Container, Title, Grid, Paper, Text, Stack, Group, Badge } from '@mantine/core';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';

export default function HomePage() {
  return (
    <AuthGuard>
      <AppLayout>
        <Container fluid>
          <Title order={2} mb="xl">Dashboard</Title>

          <Grid>
            <Grid.Col span={{ base: 12, md: 6 }}>
              <Paper p="md" withBorder>
                <Title order={4} mb="md">Recent Jumps</Title>
                <Text c="dimmed">No jumps recorded yet</Text>
              </Paper>
            </Grid.Col>

            <Grid.Col span={{ base: 12, md: 6 }}>
              <Paper p="md" withBorder>
                <Title order={4} mb="md">My Devices</Title>
                <Text c="dimmed">No devices registered</Text>
              </Paper>
            </Grid.Col>

            <Grid.Col span={12}>
              <Paper p="md" withBorder>
                <Title order={4} mb="md">Formation Skydives</Title>
                <Text c="dimmed">No formation skydives recorded</Text>
              </Paper>
            </Grid.Col>
          </Grid>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
