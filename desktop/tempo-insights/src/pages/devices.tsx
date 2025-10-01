import { useState, useEffect, useRef } from 'react';
import {
  Container,
  Title,
  Paper,
  Table,
  Text,
  Badge,
  Group,
  Button,
  ActionIcon,
  TextInput,
  Box,
  Indicator,
  Menu,
  Modal,
  Stack
} from '@mantine/core';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { IconSearch, IconTrash, IconRefresh, IconUserPlus, IconDotsVertical, IconBulb, IconSettings } from '@tabler/icons-react';
import { useRouter } from 'next/router';
import { notifications } from '@mantine/notifications';
import { AssignDeviceModal } from '../components/AssignDeviceModal';

interface Device {
  id: string;
  bluetoothId: string;
  name: string;
  state: 'ACTIVE' | 'INACTIVE' | 'PROVISIONING';
  lastSeen: string | null;
  isOnline: boolean;
  jumpCount: number;
  owner: {
    id: string;
    name: string;
    email: string;
    slug: string;
  };
  lentTo?: {
    id: string;
    name: string;
    email: string;
    slug: string;
  } | null;
}

export default function DevicesPage() {
  const router = useRouter();
  const [devices, setDevices] = useState<Device[]>([]);
  const [loading, setLoading] = useState(true);
  const [refreshing, setRefreshing] = useState(false);
  const [search, setSearch] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [lastUpdate, setLastUpdate] = useState<Date>(new Date());
  const intervalRef = useRef<NodeJS.Timeout | null>(null);
  const [assignModalOpened, setAssignModalOpened] = useState(false);
  const [selectedDevice, setSelectedDevice] = useState<Device | null>(null);
  const [confirmUnprovision, setConfirmUnprovision] = useState<Device | null>(null);

  useEffect(() => {
    checkAdminAccess();

    // Cleanup interval on unmount
    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, []);

  const checkAdminAccess = async () => {
    try {
      const response = await fetch('/api/auth/me');
      if (!response.ok) {
        router.push('/login');
        return;
      }

      const data = await response.json();
      if (data.user.role !== 'ADMIN' && data.user.role !== 'SUPER_ADMIN') {
        router.push('/home');
        return;
      }

      // If admin, load devices and start polling
      loadDevices();
      startPolling();
    } catch (err) {
      router.push('/login');
    }
  };

  const startPolling = () => {
    // Clear any existing interval
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
    }

    // Set up new interval for 15 seconds
    intervalRef.current = setInterval(() => {
      loadDevices(false, true); // silent update
    }, 15000);
  };

  const loadDevices = async (showNotification = false, silent = false) => {
    try {
      if (showNotification && !silent) {
        setRefreshing(true);
      }

      const response = await fetch('/api/devices/list');
      if (!response.ok) throw new Error('Failed to load devices');

      const data = await response.json();
      setDevices(data.devices);
      setLastUpdate(new Date());

      if (showNotification && !silent) {
        notifications.show({
          title: 'Devices refreshed',
          message: `Found ${data.devices.length} devices`,
          color: 'blue',
        });
      }
    } catch (err) {
      setError('Failed to load devices: ' + (err instanceof Error ? err.message : 'Unknown error')  );
      if (!silent) {
        notifications.show({
          title: 'Error',
          message: 'Failed to load devices',
          color: 'red',
        });
      }
    } finally {
      if (!silent) {
        setLoading(false);
        setRefreshing(false);
      }
    }
  };

  const handleRefresh = () => {
    loadDevices(true);
    // Reset polling interval
    startPolling();
  };

  const handleAssign = (device: Device) => {
    // Check if device is already in PROVISIONING state
    if (device.state === 'PROVISIONING') {
      notifications.show({
        title: 'Device Not Ready',
        message: 'This device is still in provisioning state. Please initialize it first.',
        color: 'orange',
      });
      return;
    }
    
    setSelectedDevice(device);
    setAssignModalOpened(true);
  };

  const handleBlink = async (device: Device) => {
    try {
      const response = await fetch(`/api/devices/${device.id}/blink`, {
        method: 'POST',
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to blink device');
      }

      notifications.show({
        title: 'Blink Command Sent',
        message: data.message,
        color: 'green',
      });

      // Reload devices to show updated lastSeen
      loadDevices(false, true);
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to blink device',
        color: 'red',
      });
    }
  }

  const handleUnprovision = async () => {
    if (!confirmUnprovision) return;

    try {
      const response = await fetch(`/api/devices/${confirmUnprovision.id}/commands/unprovision`, {
        method: 'POST',
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to unprovision device');
      }

      notifications.show({
        title: 'Unprovision Command Queued',
        message: data.message,
        color: 'orange',
      });

      // Close confirmation and reload devices
      setConfirmUnprovision(null);
      loadDevices(false);
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to unprovision device',
        color: 'red',
      });
    }
  };

  const handleInitialize = async (device: Device) => {
    try {
      const response = await fetch(`/api/devices/${device.id}/commands/initialize`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ pcbVersion: '1.0' })
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to initialize device');
      }

      notifications.show({
        title: 'Initialize Command Queued',
        message: `Device will be initialized as ${data.newName}`,
        color: 'blue',
      });

      // Reload devices to show updated state
      loadDevices(false);
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to initialize device',
        color: 'red',
      });
    }
  };

  const getStateBadgeColor = (state: Device['state']) => {
    switch (state) {
      case 'ACTIVE': return 'green';
      case 'INACTIVE': return 'gray';
      case 'PROVISIONING': return 'blue';
    }
  };

  const filteredDevices = devices.filter(device =>
    device.name.toLowerCase().includes(search.toLowerCase()) ||
    device.bluetoothId.toLowerCase().includes(search.toLowerCase()) ||
    device.owner.name.toLowerCase().includes(search.toLowerCase())
  );

  return (
    <AuthGuard>
      <AppLayout>
        <Container fluid>
          <Group justify="space-between" mb="xl">
            <div>
              <Title order={2}>Devices (Admin)</Title>
              <Text size="xs" c="dimmed">
                Last updated: {lastUpdate.toLocaleTimeString()}
              </Text>
            </div>
            <Button
              leftSection={<IconRefresh size={16} />}
              onClick={handleRefresh}
              loading={refreshing}
            >
              Scan for Devices
            </Button>
          </Group>

          <Paper p="md" withBorder>
            <Group mb="md">
              <TextInput
                placeholder="Search devices..."
                leftSection={<IconSearch size={16} />}
                value={search}
                onChange={(e) => setSearch(e.currentTarget.value)}
                style={{ flex: 1 }}
              />
            </Group>

            {loading ? (
              <Text>Loading devices...</Text>
            ) : error ? (
              <Text c="red">{error}</Text>
            ) : filteredDevices.length === 0 ? (
              <Text c="dimmed" ta="center" py="xl">
                {devices.length === 0
                  ? 'No devices found. Click "Scan for Devices" to discover nearby devices.'
                  : 'No devices match your search.'}
              </Text>
            ) : (
              <Table>
                <Table.Thead>
                  <Table.Tr>
                    <Table.Th>Status</Table.Th>
                    <Table.Th>Device Name</Table.Th>
                    <Table.Th>Bluetooth ID</Table.Th>
                    <Table.Th>State</Table.Th>
                    <Table.Th>Owner</Table.Th>
                    <Table.Th>Lent To</Table.Th>
                    <Table.Th>Jumps</Table.Th>
                    <Table.Th>Last Seen</Table.Th>
                    <Table.Th>Actions</Table.Th>
                  </Table.Tr>
                </Table.Thead>
                <Table.Tbody>
                  {filteredDevices.map((device) => (
                    <Table.Tr key={device.id}>
                      <Table.Td>
                        <Box>
                          <Indicator
                            color={device.isOnline ? 'green' : 'gray'}
                            size={10}
                            processing={device.isOnline}
                          >
                            <Box w={10} h={10} />
                          </Indicator>
                        </Box>
                      </Table.Td>
                      <Table.Td>
                        <Text fw={500}>{device.name}</Text>
                      </Table.Td>
                      <Table.Td>
                        <Text size="sm" style={{ fontFamily: 'monospace' }}>
                          {device.bluetoothId}
                        </Text>
                      </Table.Td>
                      <Table.Td>
                        <Badge color={getStateBadgeColor(device.state)}>
                          {device.state}
                        </Badge>
                      </Table.Td>
                      <Table.Td>{device.owner.name}</Table.Td>
                      <Table.Td>
                        {device.lentTo ? device.lentTo.name : '-'}
                      </Table.Td>
                      <Table.Td>
                        <Text ta="center">{device.jumpCount}</Text>
                      </Table.Td>
                      <Table.Td>
                        {device.lastSeen
                          ? new Date(device.lastSeen).toLocaleString()
                          : 'Never'
                        }
                      </Table.Td>
                      <Table.Td>
                        <Menu>
                          <Menu.Target>
                            <ActionIcon variant="subtle">
                              <IconDotsVertical size={16} />
                            </ActionIcon>
                          </Menu.Target>

                           <Menu.Dropdown>
                            {device.state === 'PROVISIONING' || device.name === 'Tempo-BT' ? (
                              <Menu.Item
                                leftSection={<IconSettings size={14} />}
                                onClick={() => handleInitialize(device)}
                              >
                                Initialize Device
                              </Menu.Item>
                            ) : (
                              <>
                                <Menu.Item
                                  leftSection={<IconUserPlus size={14} />}
                                  onClick={() => handleAssign(device)}
                                >
                                  Assign to User
                                </Menu.Item>
                                <Menu.Item
                                  leftSection={<IconBulb size={14} />}
                                  onClick={() => handleBlink(device)}
                                >
                                  Blink Device
                                </Menu.Item>
                                <Menu.Item
                                  leftSection={<IconTrash size={14} />}
                                  color="red"
                                  onClick={() => setConfirmUnprovision(device)}
                                >
                                  Unprovision
                                </Menu.Item>
                              </>
                            )}
                          </Menu.Dropdown>
                        </Menu>
                      </Table.Td>
                    </Table.Tr>
                  ))}
                </Table.Tbody>
              </Table>
            )}
          </Paper>

          <Modal
            opened={!!confirmUnprovision}
            onClose={() => setConfirmUnprovision(null)}
            title="Confirm Unprovision"
            centered
            size="sm"
          >
            <Stack>
              <Text>
                Are you sure you want to unprovision{' '}
                <Text span fw={700}>{confirmUnprovision?.name}</Text>?
              </Text>
              <Text size="sm" c="dimmed">
                This will reset the device state to PROVISIONING and remove any lending assignments.
                The device will need to be set up again.
              </Text>
              <Group justify="flex-end" mt="md">
                <Button variant="subtle" onClick={() => setConfirmUnprovision(null)}>
                  Cancel
                </Button>
                <Button color="red" onClick={handleUnprovision}>
                  Unprovision Device
                </Button>
              </Group>
            </Stack>
          </Modal>
          <AssignDeviceModal
            opened={assignModalOpened}
            onClose={() => setAssignModalOpened(false)}
            device={selectedDevice}
            onSuccess={async (userId: string) => {
              if (!selectedDevice) return;
              
              try {
                // Get the selected user's next jump number
                const userResponse = await fetch(`/api/users/${userId}`);
                let nextJumpNumber = 1;
                
                if (userResponse.ok) {
                  const userData = await userResponse.json();
                  nextJumpNumber = userData.user?.nextJumpNumber || 1;
                }
                
                const response = await fetch(`/api/devices/${selectedDevice.id}/commands/assign`, {
                  method: 'POST',
                  headers: { 'Content-Type': 'application/json' },
                  body: JSON.stringify({ 
                    userId,
                    nextJumpNumber
                  })
                });

                const data = await response.json();

                if (!response.ok) {
                  throw new Error(data.error || 'Failed to assign device');
                }

                notifications.show({
                  title: 'Device Assignment Queued',
                  message: data.message,
                  color: 'green',
                });

                setAssignModalOpened(false);
                setSelectedDevice(null);
                loadDevices(false);
              } catch (error) {
                notifications.show({
                  title: 'Error',
                  message: error instanceof Error ? error.message : 'Failed to assign device',
                  color: 'red',
                });
              }
            }}
          />
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
