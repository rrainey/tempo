// pages/home.tsx
import { Container, Title, Grid, Paper, Text, Stack, Group, Badge, Button } from '@mantine/core';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { MyJumpsPanel } from '../components/home/MyJumpsPanel';
import { FormationJumpsPanel } from '../components/home/FormationJumpsPanel';
import { JumpDetailsPanel } from '../components/home/JumpDetailsPanel';
import { ImportJumpModal } from '../components/home/ImportJumpModal';
import { LendDeviceForm } from '../components/LendDeviceForm';
import { useEffect, useState } from 'react';
import { notifications } from '@mantine/notifications';
import { IconCheck, IconX, IconHandStop  } from '@tabler/icons-react';
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
  const [selectedJumpId, setSelectedJumpId] = useState<string | null>(null);
  const [lendDeviceOpened, setLendDeviceOpened] = useState(false);
  const [importModalOpened, setImportModalOpened] = useState(false);
  const [refreshKey, setRefreshKey] = useState(0);

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

      setInvitations(prev => prev.filter(inv => inv.code !== code));

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

  const handleJumpSelect = (jumpId: string) => {
    setSelectedJumpId(jumpId);
  };

  const handleFormationSelect = (formationId: string) => {
    router.push(`/review/fs/${formationId}`);
  };

  const handleImportComplete = () => {
    // Trigger a refresh of the jumps panel
    setRefreshKey(prev => prev + 1);
  };

  return (
    <AuthGuard>
      <AppLayout>
        <Container fluid>
          <Group justify="space-between" mb="xl">
          <Title order={2} mb="xl">Dashboard</Title>
            <Button 
              leftSection={<IconHandStop size={16} />}
              onClick={() => setLendDeviceOpened(true)}
            >
              Lend My Device
            </Button>
          </Group>

          {/* Pending Invitations */}
          {invitations.length > 0 && (
            <Paper p="md" withBorder mb="xl" style={{ backgroundColor: 'var(--mantine-color-dark-8)' }}>
              <Title order={4} mb="md">Pending Invitations</Title>
              <Stack gap="sm">
                {invitations.map((invitation) => (
                  <Group key={invitation.id} justify="space-between">
                    <div>
                      <Text fw={500}>
                        {invitation.group.name ? (
                          <>Join group: {invitation.group.name}</>
                        ) : (
                          'System invitation'
                        )}
                      </Text>
                      <Text size="sm" c="dimmed">
                        From: {invitation.invitedBy} · 
                        Expires: {new Date(invitation.expiresAt).toLocaleDateString()}
                      </Text>
                    </div>
                    <Group gap="xs">
                      <Button
                        size="xs"
                        leftSection={<IconCheck size={14} />}
                        loading={processingInvite === invitation.code}
                        onClick={() => handleInvitation(invitation.code, true)}
                      >
                        Accept
                      </Button>
                      <Button
                        size="xs"
                        color="red"
                        variant="subtle"
                        leftSection={<IconX size={14} />}
                        loading={processingInvite === invitation.code}
                        onClick={() => handleInvitation(invitation.code, false)}
                      >
                        Decline
                      </Button>
                    </Group>
                  </Group>
                ))}
              </Stack>
            </Paper>
          )}

          <Grid>
            {/* Center Column - Jump Details */}
            <Grid.Col span={{ base: 12, md: 8 }}>
              <Paper p="md" withBorder style={{ minHeight: '400px' }}>
                <JumpDetailsPanel jumpId={selectedJumpId} />
              </Paper>
            </Grid.Col>

            {/* Right Column - Recent Activity */}
            <Grid.Col span={{ base: 12, md: 4 }}>
              <Stack gap="md">
                <MyJumpsPanel 
                  key={`jumps-${refreshKey}`}
                  onJumpSelect={handleJumpSelect}
                  onImportClick={() => setImportModalOpened(true)}
                />
                <FormationJumpsPanel onFormationSelect={handleFormationSelect} />
              </Stack>
            </Grid.Col>
          </Grid>
        </Container>
        <LendDeviceForm 
          opened={lendDeviceOpened}
          onClose={() => setLendDeviceOpened(false)}
          onSuccess={() => {
            // Optionally refresh data
          }}
        />
        <ImportJumpModal
          opened={importModalOpened}
          onClose={() => setImportModalOpened(false)}
          onImportComplete={handleImportComplete}
        />
      </AppLayout>
    </AuthGuard>
  );
}