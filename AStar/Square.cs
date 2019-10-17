using System;
using System.Collections.Generic;

namespace AStar
{
    public class Square : AStarPoint
    {
        public int X { get; set; }
        public int Y { get; set; }
        public int Weight { get; set; }
        public IEnumerable<AStarPoint> Neighbours { get; set; }

        public override IEnumerable<AStarPoint> GetNeighbors()
        {
            return Neighbours;
        }

        public override double GetHeuristicCost(AStarPoint goal)
        {
            var goalSquare = (Square)goal;
            return GetManhattanDist(X, Y, goalSquare.X, goalSquare.Y);
        }

        public override double GetCost(AStarPoint goal)
        {
            return (goal as Square).Weight * GetHeuristicCost(goal);
        }

        static int GetManhattanDist(int x1, int y1, int x2, int y2)
        {
            return Math.Abs(x1 - x2) + Math.Abs(y1 - y2);
        }

        static int GetManhattanDist(Square p1, Square p2)
        {
            return GetManhattanDist(p1.X, p1.Y, p2.X, p2.Y);
        }

        public override string ToString()
        {
            return $"{Y}, {X}";
        }
    }
}