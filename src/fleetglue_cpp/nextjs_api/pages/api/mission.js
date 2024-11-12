let missionData = null;

export default function handler(req, res) {
    if (req.method === 'POST') {
        // Validation middleware for mission data
        const { mission_data } = req.body;
        if (typeof mission_data !== 'string' || mission_data.trim() === '') {
            return res.status(400).json({ error: 'Invalid mission_data: must be a non-empty string' });
        }
        
        missionData = req.body;
        console.log('Mission data received:', missionData);
        res.setHeader('Content-Type', 'application/json');  // Set JSON content type
        res.status(200).json({ message: 'Mission data received successfully', data: missionData });
        
    } else if (req.method === 'GET') {
        res.setHeader('Content-Type', 'application/json');  // Set JSON content type
        if (missionData) {
            res.status(200).json({ mission_data: missionData.mission_data });
        } else {
            res.status(404).json({ message: 'No mission data available' });
        }
    } else {
        res.setHeader('Allow', ['GET', 'POST']);
        res.status(405).end(`Method ${req.method} Not Allowed`);
    }
}
