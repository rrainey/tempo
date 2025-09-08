import { Modal, Textarea, Button, Group, Text } from '@mantine/core';
import { useForm } from '@mantine/form';
import { notifications } from '@mantine/notifications';
import { useState, useEffect } from 'react';

interface EditNotesModalProps {
  opened: boolean;
  onClose: () => void;
  jumpId: string;
  currentNotes: string | null;
  onSuccess?: (notes: string | null) => void;
}

export function EditNotesModal({
  opened,
  onClose,
  jumpId,
  currentNotes,
  onSuccess
}: EditNotesModalProps) {
  const [loading, setLoading] = useState(false);

  const form = useForm({
    initialValues: {
      notes: currentNotes || '',
    },
  });

  useEffect(() => {
    if (opened) {
      form.setFieldValue('notes', currentNotes || '');
    }
  }, [opened, currentNotes]);

  const handleSubmit = async (values: { notes: string }) => {
    setLoading(true);

    try {
      const response = await fetch(`/api/jumps/${jumpId}/notes`, {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          notes: values.notes.trim() || null,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to update notes');
      }

      notifications.show({
        title: 'Success',
        message: 'Notes updated successfully',
        color: 'green',
      });

      onSuccess?.(data.jump.notes);
      onClose();
    } catch (error) {
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to update notes',
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
      title="Edit Jump Notes"
      size="lg"
      centered
    >
      <form onSubmit={form.onSubmit(handleSubmit)}>
        <Textarea
          placeholder="Add notes about this jump..."
          minRows={5}
          maxRows={15}
          {...form.getInputProps('notes')}
        />

        <Text size="xs" c="dimmed" mt="xs">
          Supports markdown formatting
        </Text>

        <Group justify="flex-end" mt="md">
          <Button variant="subtle" onClick={onClose}>
            Cancel
          </Button>
          <Button type="submit" loading={loading}>
            Save Notes
          </Button>
        </Group>
      </form>
    </Modal>
  );
}
