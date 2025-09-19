// components/LendDeviceForm.tsx
import { useState, useEffect } from 'react';
import {
  Modal,
  TextInput,
  Select,
  Radio,
  Group,
  Button,
  Text,
  Stack,
  Alert,
  Autocomplete,
  Loader,
  Paper,
  CopyButton,
  Box,
  Center,
} from '@mantine/core';
import { notifications } from '@mantine/notifications';
import { IconAlertCircle, IconCheck, IconCopy } from '@tabler/icons-react';
import QRCode from 'qrcode';

interface Device {
  id: string;
  name: string;
  bluetoothId: string;
  lentTo?: {
    name: string;
  };
}

interface User {
  id: string;
  name: string;
  email: string;
  slug: string;
}

interface LendDeviceFormProps {
  opened: boolean;
  onClose: () => void;
  onSuccess?: () => void;
}

export function LendDeviceForm({ opened, onClose, onSuccess }: LendDeviceFormProps) {
  const [devices, setDevices] = useState<Device[]>([]);
  const [users, setUsers] = useState<User[]>([]);
  const [selectedDevice, setSelectedDevice] = useState<string | null>(null);
  const [lendToType, setLendToType] = useState<'existing' | 'new'>('existing');
  const [selectedUser, setSelectedUser] = useState<string>('');
  const [newUserName, setNewUserName] = useState('');
  const [duration, setDuration] = useState<string>('ONE_JUMP');
  const [loading, setLoading] = useState(false);
  const [loadingData, setLoadingData] = useState(true);
  
  // New state for showing invitation after lending
  const [showInvitation, setShowInvitation] = useState(false);
  const [invitationData, setInvitationData] = useState<{
    proxyUser: { name: string; id: string };
    claimUrl: string;
    qrCodeDataUrl: string;
  } | null>(null);

  useEffect(() => {
    if (opened) {
      loadData();
    }
  }, [opened]);

  const loadData = async () => {
    setLoadingData(true);
    try {
      // Load user's devices
      const devicesResponse = await fetch('/api/devices/mine');
      if (devicesResponse.ok) {
        const data = await devicesResponse.json();
        setDevices(data.devices.filter((d: Device) => !d.lentTo));
      }

      // Load users for autocomplete
      const usersResponse = await fetch('/api/users/search');
      if (usersResponse.ok) {
        const data = await usersResponse.json();
        setUsers(data.users);
      }
    } catch (error) {
      console.error('Error loading data:', error);
    } finally {
      setLoadingData(false);
    }
  };

  const handleSubmit = async () => {
    if (!selectedDevice) {
      notifications.show({
        title: 'Error',
        message: 'Please select a device',
        color: 'red',
      });
      return;
    }

    if (lendToType === 'existing' && !selectedUser) {
      notifications.show({
        title: 'Error',
        message: 'Please select a user',
        color: 'red',
      });
      return;
    }

    if (lendToType === 'new' && !newUserName.trim()) {
      notifications.show({
        title: 'Error',
        message: 'Please enter a name for the new user',
        color: 'red',
      });
      return;
    }

    setLoading(true);
    try {
      const response = await fetch('/api/lending/lend', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          deviceId: selectedDevice,
          ...(lendToType === 'existing'
            ? { targetUserId: selectedUser }
            : { newUserName: newUserName.trim() }),
          duration,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to lend device');
      }

      notifications.show({
        title: 'Success',
        message: data.message,
        color: 'green',
      });

      // If a new proxy user was created, generate invitation
      if (lendToType === 'new' && data.device.lentTo?.isProxy) {
        await generateInvitation(data.device.lentTo);
      } else {
        onSuccess?.();
        handleClose();
      }
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to lend device',
        color: 'red',
      });
      setLoading(false);
    }
  };

  const generateInvitation = async (proxyUser: { id: string; name: string }) => {
    try {
      const response = await fetch('/api/invitations/create', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          proxyUserId: proxyUser.id,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to create invitation');
      }

      // Generate QR code
      const qrDataUrl = await QRCode.toDataURL(data.claimUrl, {
        width: 256,
        margin: 2,
        color: {
          dark: '#002233',
          light: '#ffffff',
        },
      });

      setInvitationData({
        proxyUser,
        claimUrl: data.claimUrl,
        qrCodeDataUrl: qrDataUrl,
      });
      setShowInvitation(true);
    } catch (error) {
      console.error('Failed to generate invitation:', error);
      // Even if invitation fails, the lending was successful
      onSuccess?.();
      handleClose();
    } finally {
      setLoading(false);
    }
  };

  const handleClose = () => {
    setSelectedDevice(null);
    setLendToType('existing');
    setSelectedUser('');
    setNewUserName('');
    setDuration('ONE_JUMP');
    setShowInvitation(false);
    setInvitationData(null);
    onClose();
  };

  const userOptions = users.map((user) => ({
    value: user.id,
    label: `${user.name} (${user.email})`,
  }));

  // Show invitation screen if we have invitation data
  if (showInvitation && invitationData) {
    return (
      <Modal
        opened={opened}
        onClose={handleClose}
        title="Device Lent Successfully!"
        size="md"
      >
        <Stack gap="md">
          <Alert icon={<IconAlertCircle size={16} />} color="green">
            Device lent to {invitationData.proxyUser.name}. Share this QR code or link so they can create their account.
          </Alert>

          <Paper p="md" withBorder>
            <Stack align="center">
              <Text size="sm" fw={500}>Scan to Create Account</Text>
              <Box>
                <img
                  src={invitationData.qrCodeDataUrl}
                  alt="QR Code"
                  style={{ width: '256px', height: '256px' }}
                />
              </Box>
            </Stack>
          </Paper>

          <Paper p="sm" withBorder>
            <Stack gap="xs">
              <Text size="sm" fw={500}>Or share this link:</Text>
              <Group gap="xs">
                <Text
                  size="xs"
                  style={{
                    flex: 1,
                    wordBreak: 'break-all',
                    fontFamily: 'monospace',
                  }}
                >
                  {invitationData.claimUrl}
                </Text>
                <CopyButton value={invitationData.claimUrl}>
                  {({ copied, copy }) => (
                    <Button
                      size="xs"
                      variant="subtle"
                      onClick={copy}
                      leftSection={copied ? <IconCheck size={14} /> : <IconCopy size={14} />}
                    >
                      {copied ? 'Copied' : 'Copy'}
                    </Button>
                  )}
                </CopyButton>
              </Group>
            </Stack>
          </Paper>

          <Button 
            fullWidth 
            onClick={() => {
              onSuccess?.();
              handleClose();
            }}
          >
            Dismiss
          </Button>
        </Stack>
      </Modal>
    );
  }

  return (
    <Modal
      opened={opened}
      onClose={handleClose}
      title="Lend Device"
      size="md"
    >
      {loadingData ? (
        <Stack align="center" py="xl">
          <Loader />
          <Text size="sm" c="dimmed">Loading devices...</Text>
        </Stack>
      ) : devices.length === 0 ? (
        <Alert icon={<IconAlertCircle size={16} />} color="yellow">
          You don't have any devices available to lend.
        </Alert>
      ) : (
        <Stack gap="md">
          <Select
            label="Device to lend"
            placeholder="Select a device"
            value={selectedDevice}
            onChange={setSelectedDevice}
            data={devices.map((d) => ({
              value: d.id,
              label: `${d.name} (${d.bluetoothId})`,
            }))}
            required
          />

          <Radio.Group
            label="Lend to"
            value={lendToType}
            onChange={(value) => setLendToType(value as 'existing' | 'new')}
          >
            <Group mt="xs">
              <Radio value="existing" label="Existing user" />
              <Radio value="new" label="New person" />
            </Group>
          </Radio.Group>

          {lendToType === 'existing' ? (
            <Autocomplete
              label="Select user"
              placeholder="Search by name or email"
              value={selectedUser}
              onChange={setSelectedUser}
              data={userOptions}
              required
            />
          ) : (
            <TextInput
              label="Name"
              placeholder="Enter person's name"
              value={newUserName}
              onChange={(e) => setNewUserName(e.currentTarget.value)}
              required
            />
          )}

          <Radio.Group
            label="Lending duration"
            value={duration}
            onChange={setDuration}
          >
            <Stack mt="xs" gap="xs">
              <Radio value="ONE_JUMP" label="One jump (auto-return)" />
              <Radio value="UNTIL_RECLAIM" label="Until I reclaim" />
            </Stack>
          </Radio.Group>

          <Group justify="flex-end" mt="md">
            <Button variant="subtle" onClick={handleClose} disabled={loading}>
              Cancel
            </Button>
            <Button onClick={handleSubmit} loading={loading}>
              Lend Device
            </Button>
          </Group>
        </Stack>
      )}
    </Modal>
  );
}