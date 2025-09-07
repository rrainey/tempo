import { Container, Title, Text } from '@mantine/core';
import { AppLayout } from '../../../components/AppLayout';
import { AuthGuard } from '../../../components/AuthGuard';
import Link from 'next/link';

export default function FormationsListPage() {
  return (
    <AuthGuard>
      <AppLayout>
        <Container>
          <Title order={2} mb="xl">Formation Skydives</Title>
          <Text c="dimmed">
            No formations yet. For testing, visit{' '}
            <Link href="/review/fs/test-formation">
              /review/fs/test-formation
            </Link>
          </Text>
        </Container>
      </AppLayout>
    </AuthGuard>
  );
}
