using BeamStatica.Loads.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BeamStatica.Nodes.Interfaces
{
    public interface INode : IShearForceProvider, IBendingMomentProvider, IDeflectionProvider, IRotationProvider, INumeration
    {
        short DegreesOfFreedom { get; }
        ICollection<ILoad> ConcentratedForces { get; set; }
    }
}
