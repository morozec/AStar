using System;
using System.Collections.Generic;
using System.Linq;

namespace AStar
{
    public static class Calculator
    {
        /// <summary>
        /// ������ ������� ���������������
        /// </summary>
        /// <param name="start">�����, ��� ������� �������������� ������� ���������������</param>
        /// <param name="goal">������� �����. ���� null, �� ������� ��������������� �������������� �� ��������� ����� �� ���� ��������� ����� ����</param>
        /// <param name="allPoints">��� ����� ����</param>
        /// <returns>������� ���������������</returns>
        private static ExpansionMatrixContainer GetExpansionMatrix(AStarPoint start, AStarPoint goal,
            IEnumerable<AStarPoint> allPoints)
        {
            foreach (var point in allPoints)
            {
                point.CameFromPoint = null;
            }

            var emc = new ExpansionMatrixContainer
            {
                ExpansionMatrix = new Dictionary<AStarPoint, double>(),
                //Path =  new Dictionary<Point, IList<Point>>()
            };

            var closedSet = new HashSet<AStarPoint>();
            var openSet = new HashSet<AStarPoint> {start};

            start.G = 0d;
            start.H = goal == null ? 0d : start.GetHeuristicCost(goal);

            var pathFound = false;

            while (openSet.Count > 0)
            {
                var x = GetPointWithMinF(openSet);
                if (goal != null && x == goal)
                {
                    pathFound = true;
                    break;
                }

                openSet.Remove(x);
                closedSet.Add(x);
                emc.ExpansionMatrix.Add(x, x.G);
                //emc.Path.Add(x, ReconstructPath(x));

                var neighbors = x.GetNeighbors();
                foreach (var y in neighbors)
                {
                    if (closedSet.Contains(y)) continue;

                    var tentativeGScore = x.G + x.GetCost(y);
                    bool tentativeIsBetter;

                    if (!openSet.Contains(y))
                    {
                        openSet.Add(y);
                        tentativeIsBetter = true;
                    }
                    else
                    {
                        tentativeIsBetter = tentativeGScore < y.G;
                    }

                    if (tentativeIsBetter)
                    {
                        y.CameFromPoint = x;
                        y.G = tentativeGScore;
                        y.H = goal == null ? 0d : y.GetHeuristicCost(goal);
                    }
                }
            }

            if (goal != null && !pathFound) throw new Exception("���� �� �������� ����� �� ������");


            return emc;
        }

        /// <summary>
        /// ������ ������������ ���� �� ������� �����
        /// </summary>
        /// <param name="start">��������� ����� ����</param>
        /// <param name="goal">������� ����� ����</param>
        /// <param name="allPoints">��� ����� ����</param>
        /// <returns>����������� ���� �� ��������� ����� �� �������</returns>
        public static IList<AStarPoint> GetPath(AStarPoint start, AStarPoint goal, IEnumerable<AStarPoint> allPoints)
        {
            GetExpansionMatrix(start, goal, allPoints);
            return ReconstructPath(goal);
        }

        /// <summary>
        /// ��������� ������ ��������������� ��� ������ ��������� �����
        /// </summary>
        /// <param name="startPoints">����� ��������� �����</param>
        /// <param name="allPoints">��� ����� ����</param>
        /// <returns>������� ��������������� ��� ��������� �����</returns>
        public static IDictionary<AStarPoint, ExpansionMatrixContainer> GetExpansionMatrices(
            IEnumerable<AStarPoint> startPoints, IEnumerable<AStarPoint> allPoints)
        {
            var result = new Dictionary<AStarPoint, ExpansionMatrixContainer>();
            foreach (var startPoint in startPoints)
            {
                result.Add(startPoint, GetExpansionMatrix(startPoint, null, allPoints));
            }

            return result;
        }

        /// <summary>
        /// ��������� ����� � ����������� ��������� ����� ������������ ������ ���������������
        /// </summary>
        /// <param name="expansionMatrices">������� ���������������</param>
        /// <param name="allPoints">��� ����� ����</param>
        /// <returns>����� � ����������� ������</returns>
        private static AStarPoint GetMinCostPoint(IDictionary<AStarPoint, ExpansionMatrixContainer> expansionMatrices,
            IEnumerable<AStarPoint> allPoints)
        {

            var summCosts = new Dictionary<AStarPoint, double>();
            foreach (var matrixPoint in allPoints)
            {
                summCosts.Add(matrixPoint, 0d);
                foreach (var startPoint in expansionMatrices.Keys)
                {
                    summCosts[matrixPoint] += expansionMatrices[startPoint].ExpansionMatrix[matrixPoint];
                }
            }

            AStarPoint cps = null;
            var summCost = double.MaxValue;
            foreach (var matrixPoint in summCosts.Keys)
            {
                if (summCosts[matrixPoint] < summCost)
                {
                    cps = matrixPoint;
                    summCost = summCosts[matrixPoint];
                }
            }

            return cps;
        }

        /// <summary>
        /// ��������� ����� � ����������� ��������� ������� �� (� ��) ������� 
        /// </summary>
        /// <param name="expansionMatrices">������� ���������������</param>
        /// <param name="notTraversedStartPoints">������ ������������ �����. ����� ��� ����� ����������� ����� �����������</param>
        /// <param name="collectionPoint">������� �����</param>
        /// <returns>����� � ����������� ��������� �������</returns>
        private static AStarPoint GetNearestPoint(
            IDictionary<AStarPoint, ExpansionMatrixContainer> expansionMatrices,
            IEnumerable<AStarPoint> notTraversedStartPoints,
            AStarPoint collectionPoint)
        {
            AStarPoint nearestPoint = null;
            var minCost = double.MaxValue;

            foreach (var point in notTraversedStartPoints)
            {
                if (expansionMatrices[point].ExpansionMatrix[collectionPoint] < minCost)
                {
                    nearestPoint = point;
                    minCost = expansionMatrices[point].ExpansionMatrix[collectionPoint];
                }
            }

            return nearestPoint;
        }

        /// <summary>
        /// ���������� ���������� ������������
        /// </summary>
        /// <param name="startPoints">��������� (��������) �����</param>
        /// <param name="allPoints">��� ����� ����</param>
        /// <returns>����������� ��������� ������������</returns>
        public static Infrastructure GetInfrastructure(IEnumerable<AStarPoint> startPoints,
            IEnumerable<AStarPoint> allPoints)
        {
            var channels = new List<IList<AStarPoint>>();

            //������ ������� ��������������� ��� ������
            var allExpansionMatrices = GetExpansionMatrices(startPoints, allPoints);
            //���������� ���������� ���
            var cps = GetMinCostPoint(allExpansionMatrices, allPoints);
            //������ ������� ��������������� ��� ���
            var cpsExpansionMatrixConteiner = GetExpansionMatrix(cps, null, allPoints);

            var notTraversedStartPoints = startPoints.ToList();

            //��������� ������� ��������������� ��� ��� � ������� ���� ����������� ������ ���������������
            if (!allExpansionMatrices.ContainsKey(cps))
                allExpansionMatrices.Add(cps, cpsExpansionMatrixConteiner);
            //��� ���������� ������� �� ������ ���������� ������, ���� �� � ��� ����
            if (notTraversedStartPoints.Contains(cps))
                notTraversedStartPoints.Remove(cps);

            //������� ������� (�� ���������) � ��� �����
            var nearestPoint = GetNearestPoint(allExpansionMatrices, notTraversedStartPoints, cps);

            if (nearestPoint == null)
                return new Infrastructure(cps, new List<IList<AStarPoint>>());

            //������ ����� �� ���������� ����� �� ���
            var path = ReconstructPath(cps, allExpansionMatrices[nearestPoint]);
            channels.Add(path);

            //������� ��������� ����� �� ������ ������������ �����
            notTraversedStartPoints.Remove(nearestPoint);

            while (notTraversedStartPoints.Any())
            {
                //������� ������� (�� ���������) � ��� ����� ����� ������������
                nearestPoint = GetNearestPoint(allExpansionMatrices, notTraversedStartPoints, cps);

                //���� �����, ���������� ����������� (�� ���������) ������
                IList<AStarPoint> insetPointChannel = null;
                var minInsetCost = double.MaxValue;
                foreach (var channel in channels)
                {
                    foreach (var point in channel)
                    {
                        var currentCost = allExpansionMatrices[nearestPoint].ExpansionMatrix[point];

                        if (currentCost < minInsetCost)
                        {
                            minInsetCost = currentCost;
                            insetPointChannel = channel;
                        }
                    }
                }

                if (insetPointChannel == null)
                    throw new Exception("����� ��� ������ �� ������");

                //���� ����� �� �������� ��������� ������ ������, �������� ����� 3 ������� 
                if (!nearestPoint.Equals(insetPointChannel.First()) && !nearestPoint.Equals(insetPointChannel.Last()))
                {
                    //������� �������� ����� ������ ��� ���� ����� - ������ � ����� ������, ��������������� �� ������� ���� �����
                    var realInsetPoint = GetMinCostPoint(
                        new Dictionary<AStarPoint, ExpansionMatrixContainer>()
                        {
                            {insetPointChannel.First(), allExpansionMatrices[insetPointChannel.First()]},
                            {insetPointChannel.Last(), allExpansionMatrices[insetPointChannel.Last()]},
                            {nearestPoint, allExpansionMatrices[nearestPoint]},
                        },
                        allPoints
                    );

                    //��������� ������� ������ ���������������
                    if (!allExpansionMatrices.ContainsKey(realInsetPoint))
                    {
                        allExpansionMatrices.Add(realInsetPoint, GetExpansionMatrix(realInsetPoint, null, allPoints));
                    }

                    //�������� ������������ ����� �� 2 (�� ����� ������), � ����� ��������� ����� �� ���������������� ����� �� ����� ������
                    channels.Remove(insetPointChannel);
                    var firstChannelPointPath = ReconstructPath(
                        realInsetPoint,
                        allExpansionMatrices[insetPointChannel.First()]);
                    var lastChannelPointPath = ReconstructPath(
                        realInsetPoint,
                        allExpansionMatrices[insetPointChannel.Last()]);
                    var nearestPointPath = ReconstructPath(
                        realInsetPoint,
                        allExpansionMatrices[nearestPoint]);
                    if (firstChannelPointPath.Count > 1)
                        channels.Add(firstChannelPointPath);
                    if (lastChannelPointPath.Count > 1)
                        channels.Add(lastChannelPointPath);
                    if (nearestPointPath.Count > 1)
                        channels.Add(nearestPointPath);
                }

                //������� �������������� ����� �� ������ ������������
                notTraversedStartPoints.Remove(nearestPoint);
            }

            return new Infrastructure(cps, channels);

        }

        /// <summary>
        /// ����� ����� � ����������� ������������� �������� (F)
        /// </summary>
        /// <param name="points">������ �����</param>
        /// <returns>����� � ����������� ������������� ��������</returns>
        private static AStarPoint GetPointWithMinF(IEnumerable<AStarPoint> points)
        {
            if (!points.Any())
            {
                throw new Exception("������ ������ �����");
            }

            var minF = double.MaxValue;
            AStarPoint resultPoint = null;
            foreach (var point in points)
            {
                if (point.F < minF)
                {
                    minF = point.F;
                    resultPoint = point;
                }
            }

            return resultPoint;
        }

        /// <summary>
        /// �������������� ������������ ����
        /// </summary>
        /// <param name="goal">������� �����</param>
        /// <returns>����������� ���� �� ������� �����</returns>
        private static IList<AStarPoint> ReconstructPath(AStarPoint goal)
        {
            var resultList = new List<AStarPoint>();

            var currentPoint = goal;

            while (currentPoint != null)
            {
                resultList.Add(currentPoint);
                currentPoint = currentPoint.CameFromPoint;
            }

            resultList.Reverse();

            return resultList;
        }

        public static IList<AStarPoint> ReconstructPath(AStarPoint goal,
            ExpansionMatrixContainer expansionMatrixContainer)
        {
            var path = new List<AStarPoint>() {goal};
            var currentPoint = goal;
            while (expansionMatrixContainer.ExpansionMatrix[currentPoint] > 0)
            {
                AStarPoint closestNeighbour = null;
                var minCost = double.MaxValue;
                foreach (var neihgbour in currentPoint.GetNeighbors())
                {
                    if (expansionMatrixContainer.ExpansionMatrix[neihgbour] < minCost)
                    {
                        minCost = expansionMatrixContainer.ExpansionMatrix[neihgbour];
                        closestNeighbour = neihgbour;
                    }
                }

                currentPoint = closestNeighbour;
                path.Add(closestNeighbour);
            }

            return path;
        }
    }
}