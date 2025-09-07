import { Container, Title, Grid, Paper, Text, Stack, Group, Badge, Button, Card } from '@mantine/core';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { useEffect, useState } from 'react';
import { notifications } from '@mantine/notifications';
import { IconCheck, IconX } from '@tabler/icons-react';
import { useRouter } from 'next/router';

interface Invitation {
  id: string;
  code: string;
  group: {
    id?: string;
    name?: string;
    slug?: string;
  };
  groupRole: string;
  invitedBy: string;
  expiresAt: string;
  createdAt: string;
}

export default function HomePage() {
  const router = useRouter();
  const [invitations, setInvitations] = useState<Invitation[]>([]);
  const [loadingInvitations, setLoadingInvitations] = useState(true);
  const [processingInvite, setProcessingInvite] = useState<string | null>(null);

  useEffect(() => {
    loadInvitations();
  }, []);

  const loadInvitations = async () => {
    try {
      const response = await fetch('/api/users/invitations');
      if (response.ok) {
        const data = await response.json();
        setInvitations(data.invitations);
      }
    } catch (error) {
      console.error('Error loading invitations:', error);
    } finally {
      setLoadingInvitations(false);
    }
  };

  const handleInvitation = async (code: string, accept: boolean) => {
    setProcessingInvite(code);

    try {
      const response = await fetch(`/api/invitations/${code}/${accept ? 'accept' : 'decline'}`, {
        method: 'POST',
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || `Failed to ${accept ? 'accept' : 'decline'} invitation`);
      }

      notifications.show({
        title: 'Success',
        message: data.message,
        color: 'green',
      });

      // Remove invitation from list
      setInvitations(prev => prev.filter(inv => inv.code !== code));

      // If accepted, redirect to group
      if (accept && data.group?.slug) {
        router.push(`/groups/${data.group.slug}`);
      }
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'An error occurred',
        color: 'red',
      });
    } finally {
      setProcessingInvite(null);
    }
  };

  return (
    <AuthGuard>
      <AppLayout>
        <Container fluid>
          <Title order={2} mb="xl">Dashboard</Title>

          {/* Pending Invitations */}
          {invitations.length > 0 && (
            <Paper p="md" withBorder mb="xl" style={{ backgroundColor: 'var(--mantine-color-dark-8)' }}>
              <Title order={4} mb="md">Pending Invitations</Title>
              <Stack gap="sm">
                {invitations.map(invitation => (
                  <Card key={invitation.id} p="sm" withBorder>
                    <Group justify="space-between" align="center">
                      <div>
                        <Text fw={500}>
                          {invitation.invitedBy} invited you to join{' '}
                          <Text span fw={700}>{invitation.group.name}</Text>
                        </Text>
                        <Group gap="xs" mt="xs">
                          <Badge size="sm">
                            Role: {invitation.groupRole}
                          </Badge>
                          <Text size="xs" c="dimmed">
                            Expires {new Date(invitation.expiresAt).toLocaleDateString()}
                          </Text>
                        </Group>
                      </div>
                      <Group gap="xs">
                        <Button
                          size="sm"
                          color="green"
                          leftSection={<IconCheck size={16} />}
                          onClick={() => handleInvitation(invitation.code, true)}
                          loading={processingInvite === invitation.code}
                          disabled={processingInvite !== null}
                        >
                          Accept
                        </Button>
                        <Button
                          size="sm"
                          variant="subtle"
                          color="red"
                          leftSection={<IconX size={16} />}
                          onClick={() => handleInvitation(invitation.code, false)}
                          loading={processingInvite === invitation.code}
                          disabled={processingInvite !== null}
                        >
                          Decline
                        </Button>
                      </Group>
                    </Group>
                  </Card>
                ))}
              </Stack>
            </Paper>
          )}

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
