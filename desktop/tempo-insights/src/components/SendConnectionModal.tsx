// src/components/SendConnectionModal.tsx
import { Modal, Button, TextInput, Stack, Text } from '@mantine/core';
import { useState } from 'react';
import { notifications } from '@mantine/notifications';
import { IconUserPlus } from '@tabler/icons-react';

interface SendConnectionModalProps {
  opened: boolean;
  onClose: () => void;
  targetUser: {
    id: string;
    name: string;
    slug: string;
  };
  onSuccess?: () => void;
}

export function SendConnectionModal({ 
  opened, 
  onClose, 
  targetUser,
  onSuccess 
}: SendConnectionModalProps) {
  const [message, setMessage] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSend = async () => {
    setLoading(true);

    try {
      const response = await fetch('/api/connections/request', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          toUserId: targetUser.id,
          message: message.trim() || undefined
        })
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to send connection request');
      }

      notifications.show({
        title: 'Success',
        message: data.message || 'Connection request sent',
        color: 'green',
      });

      onClose();
      setMessage('');
      onSuccess?.();
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'An error occurred',
        color: 'red',
      });
    } finally {
      setLoading(false);
    }
  };

  return (
    <Modal
      opened={opened}
      onClose={onClose}
      title="Send Connection Request"
      size="md"
    >
      <Stack>
        <Text>
          Send a connection request to <strong>{targetUser.name}</strong>
        </Text>
        
        <TextInput
          label="Message (optional)"
          placeholder="Add a personal message..."
          value={message}
          onChange={(e) => setMessage(e.currentTarget.value)}
          maxLength={500}
        />

        <Button
          leftSection={<IconUserPlus size={16} />}
          onClick={handleSend}
          loading={loading}
          fullWidth
        >
          Send Connection Request
        </Button>
      </Stack>
    </Modal>
  );
}