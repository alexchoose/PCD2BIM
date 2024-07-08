import { loadIfcCommon } from "./viz_common.mjs";


async function loadIfcFromColab(width="900px", height="500px") {
  google.colab.kernel.comms.registerTarget('ifcjsviz', (comm, message) => {
    let ifcStr = message["data"]["ifcStr"];
    loadIfcCommon(ifcStr, height, width, true);
    const container = document.getElementById("ifcjs-container");
    container.style.width = width;
    container.style.height = height;
  });
}

window.loadIfcFromColab = loadIfcFromColab;
