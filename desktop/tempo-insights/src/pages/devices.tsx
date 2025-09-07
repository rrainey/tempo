import { useState, useEffect } from 'react';
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
  TextInput
} from '@mantine/core';
import { AppLayout } from '../components/AppLayout';
import { AuthGuard } from '../components/AuthGuard';
import { IconSearch, IconLink, IconRefresh } from '@tabler/icons-react';
import { useRouter } from 'next/router';

interface Device {
  id: string;
  bluetoothId: string;
  name: string;
  state: 'ACTIVE' | 'INACTIVE' | 'PROVISIONING';
  lastSeen: string | null;
  owner: {
    name: string;
    email: string;
  };
  lentTo?: {
    name: string;
    email: string;
  } | null;
}

export default function DevicesPage() {
  const router = useRouter();
  const [devices, setDevices] = useState<Device[]>([]);
  const [loading, setLoading] = useState(true);
  const [search, setSearch] = useState('');
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    checkAdminAccess();
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

      // If admin, load devices (placeholder for now)
      loadDevices();
    } catch (err) {
      router.push('/login');
    }
  };

  const loadDevices = async () => {
    try {
      // TODO: Implement devices API endpoint
      // const response = await fetch('/api/devices');
      // const data = await response.json();
      // setDevices(data.devices);

      // Placeholder data for now
      setDevices([]);
      setLoading(false);
    } catch (err) {
      setError('Failed to load devices');
      setLoading(false);
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
            <Title order={2}>Devices (Admin)</Title>
            <Button leftSection={<IconRefresh size={16} />} onClick={loadDevices}>
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
            ) : devices.length === 0 ? (
              <Text c="dimmed" ta="center" py="xl">
                No devices found. Click "Scan for Devices" to discover nearby devices.
              </Text>
            ) : (
              <Table>
                <Table.Thead>
                  <Table.Tr>
                    <Table.Th>Device Name</Table.Th>
                    <Table.Th>Bluetooth ID</Table.Th>
                    <Table.Th>State</Table.Th>
                    <Table.Th>Owner</Table.Th>
                    <Table.Th>Lent To</Table.Th>
                    <Table.Th>Last Seen</Table.Th>
                    <Table.Th>Actions</Table.Th>
                  </Table.Tr>
                </Table.Thead>
                <Table.Tbody>
                  {filteredDevices.map((device) => (
                    <Table.Tr key={device.id}>
                      <Table.Td>{device.name}</Table.Td>
                      <Table.Td style={{ fontFamily: 'monospace' }}>
                        {device.bluetoothId}
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
                        {device.lastSeen
                          ? new Date(device.lastSeen).toLocaleString()
                          : 'Never'
                        }
                      </Table.Td>
                      <Table.Td>
                        <ActionIcon variant="subtle" title="Blink device">
                          <IconLink size={16} />
                        </ActionIcon>
                      </Table.Td>
                    </Table.Tr>
                  ))}
                </Table.Tbody>
              </Table>
            )}
          </Paper>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
