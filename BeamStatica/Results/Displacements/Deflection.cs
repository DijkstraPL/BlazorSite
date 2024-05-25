using BeamStatica.Results.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BeamStatica.Results.Displacements
{
    public sealed class Deflection : Displacement
    {
        public override string ToString() => Value.ToString();
    }
}
