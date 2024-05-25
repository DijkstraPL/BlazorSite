using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BeamStatica.Loads.Interfaces
{
    public interface ILoad : ILoadPosition
    {
        double CalculateShear();
        double CalculateBendingMoment(double distanceFromLoad);
    }
}
