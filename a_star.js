const fs = require('fs');

// Read track from file
const fileContent = fs.readFileSync('track.txt', 'utf8');
const trackMatrix = fileContent.trim().split('\n').map(line => line.trim());

// Convert track to coordinates
const coordinates = trackToCoordinates(trackMatrix);
const startPoint = coordinates.S[0];
const finishPoints = coordinates.F;
const obstacles = coordinates.O;
const grass = coordinates.G;

function heuristic(point, finishPoints) {
  return Math.min(...finishPoints.map(finish => distance(point, finish)));
}

function aStar(startPoint, finishPoints, obstacles) {
  let openSet = [{ point: startPoint, g: 0, f: heuristic(startPoint, finishPoints), parent: null }];
  const closedSet = new Set();

  while (openSet.length > 0) {
    // Sort the open set by the lowest f value
    openSet.sort((a, b) => a.f - b.f);
    const currentNode = openSet.shift(); // Pop the node with the lowest f value
    const currentPoint = currentNode.point;

    if (isAtFinish(currentPoint, finishPoints)) {
      return reconstructPath(currentNode);
    }

    closedSet.add(currentPoint.toString());

    const neighbors = getSurroundingPoints(currentPoint[0], currentPoint[1])
      .filter(point => !obstacleInPath(obstacles, currentPoint, point) && !closedSet.has(point.toString()));

    for (const neighbor of neighbors) {
      const tentativeG = currentNode.g + distance(currentPoint, neighbor);
      let neighborNode = openSet.find(node => node.point.toString() === neighbor.toString());

      if (neighborNode) {
        if (tentativeG < neighborNode.g) {
          neighborNode.g = tentativeG;
          neighborNode.f = tentativeG + heuristic(neighbor, finishPoints);
          neighborNode.parent = currentNode;
        }
      } else {
        neighborNode = {
          point: neighbor,
          g: tentativeG,
          f: tentativeG + heuristic(neighbor, finishPoints),
          parent: currentNode
        };
        openSet.push(neighborNode);
      }
    }
  }

  return null; // No path found
}

function isAtFinish(point, finishPoints) {
  return finishPoints.some(finish => point[0] === finish[0] && point[1] === finish[1]);
}

function reconstructPath(node) {
  const path = [];
  while (node) {
    path.push(node.point);
    node = node.parent;
  }
  return path.reverse();
}

const path = aStar(startPoint, finishPoints, obstacles);
if (path) {
  const formattedArray = path.map(innerArray => innerArray.join(','));
  const content = formattedArray.join('\n');
  fs.writeFileSync('trip.rl', content, 'utf8');
  console.log('Path found:', path);
} else {
  console.log('No path found.');
}

///////////////////////
// HELPER FUNCTIONS //
/////////////////////

function trackToCoordinates(matrix) {
  const coordinates = {};
  for (let i = matrix.length - 1; i >= 0; i--) {
    for (let j = matrix[i].length - 1; j >= 0; j--) {
      const char = matrix[i][j];
      if (!coordinates[char]) {
        coordinates[char] = [];
      }
      coordinates[char].push([j, matrix.length - 1 - i]);
    }
  }
  return coordinates;
}

function getSurroundingPoints(x, y) {
  const surroundingPoints = [];
  for (let dx = -1; dx <= 1; dx++) {
    for (let dy = -1; dy <= 1; dy++)
      if (x + dx >= 0 && y + dy >= 0)
        surroundingPoints.push([x + dx, y + dy]);
  }
  return surroundingPoints;
}

function distance(point1, point2) {
  const [x1, y1] = point1;
  const [x2, y2] = point2;
  return Math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2);
}

function obstacleInPath(obstacles, startPoint, endPoint) {
  const minX = Math.min(startPoint[0], endPoint[0]);
  const maxX = Math.max(startPoint[0], endPoint[0]);
  const minY = Math.min(startPoint[1], endPoint[1]);
  const maxY = Math.max(startPoint[1], endPoint[1]);

  for (let i = 0; i < obstacles.length; i++) {
    const obstacle = obstacles[i];
    const obstacleMinX = obstacle[0] - 0.5;
    const obstacleMaxX = obstacle[0] + 0.5;
    const obstacleMinY = obstacle[1] - 0.5;
    const obstacleMaxY = obstacle[1] + 0.5;
    if (
      minX <= obstacleMaxX &&
      maxX >= obstacleMinX &&
      minY <= obstacleMaxY &&
      maxY >= obstacleMinY
    ) {
      return true;
    }
  }
  return false;
}
