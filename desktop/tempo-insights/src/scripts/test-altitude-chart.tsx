// Save as: src/scripts/test-altitude-chart.tsx
// This is a visual test - create as a temporary page to view the chart

import { Container, Title, Stack, SimpleGrid, Text } from '@mantine/core';
import { JumpAltitudeChart } from '@/components/home/JumpAltitudeChart';

export default function TestAltitudeChart() {
  return (
    <Container size="lg" py="xl">
      <Title order={2} mb="xl">Altitude Chart Test</Title>
      
      <Stack gap="xl">
        <div>
          <Text size="lg" fw={500} mb="md">Standard Jump Profile</Text>
          <JumpAltitudeChart
            jumpId="test-1"
            exitTime={20}
            deployTime={75}
            landingTime={180}
            exitAltitude={14000}
            deployAltitude={3500}
          />
        </div>

        <div>
          <Text size="lg" fw={500} mb="md">High Altitude Jump</Text>
          <JumpAltitudeChart
            jumpId="test-2"
            exitTime={30}
            deployTime={120}
            landingTime={300}
            exitAltitude={18000}
            deployAltitude={4500}
          />
        </div>

        <div>
          <Text size="lg" fw={500} mb="md">Quick Hop & Pop</Text>
          <JumpAltitudeChart
            jumpId="test-3"
            exitTime={15}
            deployTime={25}
            landingTime={120}
            exitAltitude={5500}
            deployAltitude={4000}
          />
        </div>
      </Stack>
    </Container>
  );
}