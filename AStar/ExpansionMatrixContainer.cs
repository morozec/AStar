using System.Collections.Generic;

namespace AStar
{
    /// <summary>
    /// ������� ��������������
    /// </summary>
    public class ExpansionMatrixContainer
    {
        /// <summary>
        /// ��������� ������� �� ����� ����
        /// </summary>
        public IDictionary<AStarPoint, double> ExpansionMatrix { get; set; }
        ///// <summary>
        ///// ����������� ���� ������� �� ����� ����
        ///// </summary>
        //public IDictionary<Point, IList<Point>> Path { get; set; } 
    }
}