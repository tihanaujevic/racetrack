const fs = require('fs');
const PriorityQueue = require('js-priority-queue');

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
  // Use Manhattan distance for better performance on grid-based paths
  return Math.min(...finishPoints.map(finish => Math.abs(point[0] - finish[0]) + Math.abs(point[1] - finish[1])));
}

function aStar(startPoint, finishPoints, obstacles, grass) {
  let openSet = new PriorityQueue({ comparator: (a, b) => a.f - b.f });
  let openSetMap = new Map();

  const startNode = {
    point: startPoint,
    velocity: [0, 0],
    g: 0,
    f: heuristic(startPoint, finishPoints),
    parent: null
  };

  openSet.queue(startNode);
  openSetMap.set(stateToString(startPoint, [0, 0]), startNode);

  const closedSet = new Set();

  while (openSet.length > 0) {
    const currentNode = openSet.dequeue();
    openSetMap.delete(stateToString(currentNode.point, currentNode.velocity));
    const currentPoint = currentNode.point;
    const currentVelocity = currentNode.velocity;

    if (isAtFinish(currentPoint, finishPoints)) {
      return reconstructPath(currentNode);
    }

    closedSet.add(stateToString(currentPoint, currentVelocity));

    const neighbors = getPossibleMoves(currentPoint, currentVelocity, grass)
      .filter(([point, velocity]) =>
        !obstacleInPath(obstacles, currentPoint, point) &&
        !closedSet.has(stateToString(point, velocity))
      );

    for (const [neighbor, newVelocity] of neighbors) {
      const tentativeG = currentNode.g + 1; // Each move costs 1
      const neighborState = stateToString(neighbor, newVelocity);
      let neighborNode = openSetMap.get(neighborState);

      if (neighborNode) {
        if (tentativeG < neighborNode.g) {
          neighborNode.g = tentativeG;
          neighborNode.f = tentativeG + heuristic(neighbor, finishPoints);
          neighborNode.parent = currentNode;
          openSet.queue(neighborNode);
          openSetMap.set(neighborState, neighborNode);
        }
      } else {
        neighborNode = {
          point: neighbor,
          velocity: newVelocity,
          g: tentativeG,
          f: tentativeG + heuristic(neighbor, finishPoints),
          parent: currentNode
        };
        openSet.queue(neighborNode);
        openSetMap.set(neighborState, neighborNode);
      }
    }
  }

  return null;
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

var startTime = performance.now();
const path = aStar(startPoint, finishPoints, obstacles, grass);
var endTime = performance.now();
console.log(`A* took ${endTime - startTime} milliseconds`);

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

function getPossibleMoves([x, y], [vx, vy], grass) {
  const possibleMoves = [];
  const onGrass = grass.some(g => g[0] === x && g[1] === y);

  for (let dx = -1; dx <= 1; dx++) {
    for (let dy = -1; dy <= 1; dy++) {
      const newVelocity = [vx + dx, vy + dy];
      const newPosition = [x + newVelocity[0], y + newVelocity[1]];

      // Apply constraints for integer steps
      if (Number.isInteger(newPosition[0]) && Number.isInteger(newPosition[1])) {
        // Check if the new position is within bounds
        if (newPosition[0] >= 0 && newPosition[1] >= 0) {
          if (onGrass) {
            // If currently on grass, decelerate in the direction of motion
            if ((vx > 0 && newVelocity[0] < vx) || (vx < 0 && newVelocity[0] > vx) || (vx === 0 && newVelocity[0] === 0)) {
              if ((vy > 0 && newVelocity[1] < vy) || (vy < 0 && newVelocity[1] > vy) || (vy === 0 && newVelocity[1] === 0)) {
                possibleMoves.push([newPosition, newVelocity]);
              }
            }
          } else {
            // If not on grass, consider normal movement (acceleration, deceleration, and steering)
            if (Math.abs(newVelocity[0] - vx) <= 1 && Math.abs(newVelocity[1] - vy) <= 1) {
              possibleMoves.push([newPosition, newVelocity]);
            }
          }
        }
      }
    }
  }
  return possibleMoves;
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

function stateToString(point, velocity) {
  return `${point[0]},${point[1]},${velocity[0]},${velocity[1]}`;
}
