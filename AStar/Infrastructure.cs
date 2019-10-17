using System.Collections.Generic;

namespace AStar
{
    public class Infrastructure
    {
        /// <summary>
        /// ����� ���������� ���
        /// </summary>
        public AStarPoint Cps { get; private set; }
        /// <summary>
        /// ������ �������
        /// </summary>
        public IEnumerable<IList<AStarPoint>> Channels { get; private set; }

        public Infrastructure(AStarPoint cps, IEnumerable<IList<AStarPoint>> channels)
        {
            Cps = cps;
            Channels = channels;
        }
    }
}