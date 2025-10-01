// components/home/ImportJumpModal.tsx

import React, { useState } from 'react';
import {
  Modal,
  Button,
  Group,
  Text,
  Stack,
  Stepper,
  NumberInput,
  Textarea,
  Alert,
  Paper,
  rem,
  LoadingOverlay,
  Badge,
} from '@mantine/core';
import { Dropzone } from '@mantine/dropzone';
import { notifications } from '@mantine/notifications';
import {
  IconUpload,
  IconX,
  IconFile,
  IconCheck,
  IconAlertCircle,
  IconHash,
  IconCalendar,
  IconFileDescription,
  IconMapPin,
} from '@tabler/icons-react';

interface ImportJumpModalProps {
  opened: boolean;
  onClose: () => void;
  onImportComplete: () => void;
}

interface FileInfo {
  hash: string;
  fileName: string;
  fileSize: number;
  suggestedJumpNumber: number;
  startDate: string | null;
  startLocation: {
    lat_deg: number;
    lon_deg: number;
    alt_m: number;
  } | null;
}

export function ImportJumpModal({ opened, onClose, onImportComplete }: ImportJumpModalProps) {
  const [active, setActive] = useState(0);
  const [file, setFile] = useState<File | null>(null);
  const [uploading, setUploading] = useState(false);
  const [processing, setProcessing] = useState(false);
  const [error, setError] = useState<string | null>(null);
  
  // File info from upload
  const [fileInfo, setFileInfo] = useState<FileInfo | null>(null);
  const [alreadyExists, setAlreadyExists] = useState(false);
  
  // Form fields for confirmation
  const [jumpNumber, setJumpNumber] = useState<number | undefined>();
  const [notes, setNotes] = useState('');

  const handleFileSelect = (files: File[]) => {
    if (files.length > 0) {
      setFile(files[0]);
      setError(null);
      setAlreadyExists(false);
    }
  };

  const handleUpload = async () => {
    if (!file) return;

    setUploading(true);
    setError(null);

    try {
      const formData = new FormData();
      formData.append('file', file);

      const response = await fetch('/api/jumps/import', {
        method: 'POST',
        body: formData,
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'Failed to upload file');
      }

      const data = await response.json();
      
      if (data.alreadyExists) {
        setAlreadyExists(true);
        notifications.show({
          title: 'Duplicate File',
          message: 'This jump log already exists in your account',
          color: 'blue',
        });
        return;
      }

      setFileInfo(data.fileInfo);
      setJumpNumber(data.fileInfo.suggestedJumpNumber);
      
      // Move to next step
      setActive(1);
    } catch (error) {
      setError(error instanceof Error ? error.message : 'Upload failed');
      notifications.show({
        title: 'Upload Error',
        message: error instanceof Error ? error.message : 'Failed to upload file',
        color: 'red',
      });
    } finally {
      setUploading(false);
    }
  };

  const handleFinish = async () => {
    if (!fileInfo) return;

    setProcessing(true);
    setError(null);

    try {
      const response = await fetch('/api/jumps/import/confirm', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          hash: fileInfo.hash,
          jumpNumber: jumpNumber || undefined,
          notes: notes || null,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'Failed to save jump');
      }

      notifications.show({
        title: 'Success',
        message: 'Jump imported successfully. Processing will begin shortly.',
        color: 'green',
        icon: <IconCheck />,
      });

      // Reset and close
      handleClose();
      onImportComplete();
    } catch (error) {
      setError(error instanceof Error ? error.message : 'Save failed');
      notifications.show({
        title: 'Error',
        message: error instanceof Error ? error.message : 'Failed to save jump',
        color: 'red',
      });
    } finally {
      setProcessing(false);
    }
  };

  const handleClose = () => {
    // Reset all state
    setActive(0);
    setFile(null);
    setFileInfo(null);
    setJumpNumber(undefined);
    setNotes('');
    setError(null);
    setAlreadyExists(false);
    onClose();
  };

  const nextStep = () => setActive((current) => (current < 1 ? current + 1 : current));
  const prevStep = () => setActive((current) => (current > 0 ? current - 1 : current));

  const formatFileSize = (bytes: number) => {
    const mb = bytes / 1024 / 1024;
    return `${mb.toFixed(2)} MB`;
  };

  return (
    <Modal
      opened={opened}
      onClose={handleClose}
      title="Import a Jump Log"
      size="lg"
      closeOnClickOutside={false}
    >
      <LoadingOverlay visible={uploading || processing} />
      
      <Stepper active={active} onStepClick={setActive}>
        <Stepper.Step label="Upload File" description="Select jump log file">
          <Stack mt="xl">
            {error && (
              <Alert icon={<IconAlertCircle />} color="red" variant="light">
                {error}
              </Alert>
            )}

            {alreadyExists && (
              <Alert icon={<IconAlertCircle />} color="blue" variant="light">
                This jump log already exists in your account. Please select a different file.
              </Alert>
            )}

            {!file ? (
              <Dropzone
                onDrop={handleFileSelect}
                onReject={(files) => setError('Invalid file type')}
                maxSize={16 * 1024 * 1024} // 16MB
                accept={['application/octet-stream', 'text/plain']}
              >
                <Group justify="center" gap="xl" style={{ minHeight: rem(220), pointerEvents: 'none' }}>
                  <Dropzone.Accept>
                    <IconUpload
                      style={{ width: rem(52), height: rem(52), color: 'var(--mantine-color-green-6)' }}
                      stroke={1.5}
                    />
                  </Dropzone.Accept>
                  <Dropzone.Reject>
                    <IconX
                      style={{ width: rem(52), height: rem(52), color: 'var(--mantine-color-red-6)' }}
                      stroke={1.5}
                    />
                  </Dropzone.Reject>
                  <Dropzone.Idle>
                    <IconUpload
                      style={{ width: rem(52), height: rem(52), color: 'var(--mantine-color-dimmed)' }}
                      stroke={1.5}
                    />
                  </Dropzone.Idle>

                  <div>
                    <Text size="xl" inline>
                      Drag a log file here or click to select
                    </Text>
                    <Text size="sm" color="dimmed" inline mt={7}>
                      Attach a Tempo jump log file (.txt, or .log), max size 16MB
                    </Text>
                  </div>
                </Group>
              </Dropzone>
            ) : (
              <Paper p="md" withBorder>
                <Group>
                  <IconFile size={32} />
                  <div style={{ flex: 1 }}>
                    <Text size="sm" fw={500}>{file.name}</Text>
                    <Text size="xs" color="dimmed">
                      {formatFileSize(file.size)}
                    </Text>
                  </div>
                  <Button
                    variant="subtle"
                    color="red"
                    size="sm"
                    onClick={() => {
                      setFile(null);
                      setAlreadyExists(false);
                    }}
                  >
                    Remove
                  </Button>
                </Group>
              </Paper>
            )}
          </Stack>
        </Stepper.Step>

        <Stepper.Step label="Confirm Details" description="Review and save">
          <Stack mt="xl">
            {fileInfo && (
              <>
                {/* File Information */}
                <Paper p="md" withBorder>
                  <Text size="sm" fw={600} mb="sm">File Information</Text>
                  <Stack gap="xs">
                    <Group gap="xs">
                      <IconFileDescription size={16} />
                      <Text size="sm">Name: {fileInfo.fileName}</Text>
                    </Group>
                    <Group gap="xs">
                      <IconHash size={16} />
                      <Text size="sm">Size: {formatFileSize(fileInfo.fileSize)}</Text>
                    </Group>
                    {fileInfo.startDate && (
                      <Group gap="xs">
                        <IconCalendar size={16} />
                        <Text size="sm">
                          Jump Date: {new Date(fileInfo.startDate).toLocaleString('en-US', {
                            weekday: 'short',
                            year: 'numeric',
                            month: 'short',
                            day: 'numeric',
                            hour: 'numeric',
                            minute: '2-digit',
                            second: '2-digit',
                            hour12: true
                          })}
                        </Text>
                      </Group>
                    )}
                    {fileInfo.startLocation && (
                      <Group gap="xs">
                        <IconMapPin size={16} />
                        <Text size="sm">
                          Location: {fileInfo.startLocation.lat_deg.toFixed(5)}°, {fileInfo.startLocation.lon_deg.toFixed(5)}°
                        </Text>
                      </Group>
                    )}
                    <Badge size="sm" color="blue" variant="light">
                      Hash: {fileInfo.hash.substring(0, 16)}...
                    </Badge>
                  </Stack>
                </Paper>

                <Stack>
                  <Alert icon={<IconAlertCircle />} color="blue" variant="light">
                    {fileInfo.startDate ? (
                      <>
                        Look at the date and time shown above. If this doesn&apos;t match the jump 
                        you wish to import, you can go back and select a different file.
                      </>
                    ) : (
                      <>
                        Jump date/time could not be determined from the file. 
                        Please ensure this is the correct jump file before proceeding.
                      </>
                    )}
                  </Alert>

                  <NumberInput
                    label="Jump Number"
                    description="From your logbook (optional - will auto-increment if left blank)"
                    placeholder={`Suggested: ${fileInfo.suggestedJumpNumber}`}
                    value={jumpNumber}
                    onChange={(val) => {
                      const num = typeof val === 'number' && !isNaN(val) ? val : undefined;
                      setJumpNumber(num);
                    }}
                    min={1}
                  />

                  <Textarea
                    label="Notes"
                    description="Any notes about this jump (optional)"
                    placeholder="Enter notes..."
                    value={notes}
                    onChange={(e) => setNotes(e.currentTarget.value)}
                    minRows={3}
                    maxRows={6}
                    maxLength={5000}
                  />
                </Stack>

                <Alert icon={<IconAlertCircle />} color="blue" variant="light" mt="md">
                  After import, your jump will be automatically analyzed. Exit time, freefall time, 
                  and other metrics will appear once processing is complete.
                </Alert>
              </>
            )}
          </Stack>
        </Stepper.Step>
      </Stepper>

      <Group justify="space-between" mt="xl">
        <Button variant="subtle" onClick={handleClose}>
          Cancel
        </Button>
        <Group>
          {active > 0 && (
            <Button variant="default" onClick={prevStep}>
              Back
            </Button>
          )}
          {active === 0 && (
            <Button
              onClick={handleUpload}
              disabled={!file || uploading || alreadyExists}
              loading={uploading}
            >
              Next
            </Button>
          )}
          {active === 1 && (
            <Button
              onClick={handleFinish}
              disabled={!fileInfo || processing}
              loading={processing}
            >
              Import Jump
            </Button>
          )}
        </Group>
      </Group>
    </Modal>
  );
}