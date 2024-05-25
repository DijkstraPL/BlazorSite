using BeamStatica.Results.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BeamStatica.Results.Reactions
{
    public sealed class BendingMoment  : Reaction
    {
        public override string ToString() => Value.ToString();
    }
}
