import { ReactElement } from "react";

type P = { children: React.ReactNode };
function Svg({ children }: P): ReactElement {
  return (
    <svg
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth={1.8}
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      {children}
    </svg>
  );
}

export const IconPan = (): ReactElement => (
  <Svg>
    <path d="M12 3v18M3 12h18M12 3l-3 3M12 3l3 3M12 21l-3-3M12 21l3-3M3 12l3-3M3 12l3 3M21 12l-3-3M21 12l-3 3" />
  </Svg>
);

export const IconSafeArea = (): ReactElement => (
  <Svg>
    <path d="M5 7l6-3 8 4-1 9-9 3-5-6z" fill="currentColor" fillOpacity={0.15} />
    <circle cx="5" cy="7" r="1.4" fill="currentColor" />
    <circle cx="11" cy="4" r="1.4" fill="currentColor" />
    <circle cx="19" cy="8" r="1.4" fill="currentColor" />
    <circle cx="18" cy="17" r="1.4" fill="currentColor" />
    <circle cx="9" cy="20" r="1.4" fill="currentColor" />
  </Svg>
);

export const IconStart = (): ReactElement => (
  <Svg>
    <circle cx="12" cy="12" r="7.5" />
    <path d="M12 2.5v19M2.5 12h19" />
  </Svg>
);

export const IconGnss = (): ReactElement => (
  <Svg>
    <path d="M12 21s-6.5-5.6-6.5-11a6.5 6.5 0 0113 0c0 5.4-6.5 11-6.5 11z" />
    <circle cx="12" cy="10" r="2.3" />
  </Svg>
);

export const IconPath = (): ReactElement => (
  <Svg>
    <circle cx="5" cy="18" r="2" />
    <circle cx="12" cy="7" r="2" />
    <circle cx="19" cy="15" r="2" />
    <path d="M6.2 16.3l4.6-7.6M13.6 8.4l4 5" />
  </Svg>
);

export const IconLoop = (): ReactElement => (
  <Svg>
    <path d="M4 12a8 8 0 0113.7-5.6L20 8.5" />
    <path d="M20 4v4.5h-4.5" />
    <path d="M20 12a8 8 0 01-13.7 5.6L4 15.5" />
    <path d="M4 20v-4.5h4.5" />
  </Svg>
);

export const IconUndo = (): ReactElement => (
  <Svg>
    <path d="M9 14L4 9l5-5" />
    <path d="M4 9h10.5a5.5 5.5 0 010 11H11" />
  </Svg>
);

export const IconRedo = (): ReactElement => (
  <Svg>
    <path d="M15 14l5-5-5-5" />
    <path d="M20 9H9.5a5.5 5.5 0 000 11H13" />
  </Svg>
);

export const IconTrash = (): ReactElement => (
  <Svg>
    <path d="M4 7h16M10 11v6M14 11v6M6 7l1 13h10l1-13M9 7V4h6v3" />
  </Svg>
);

export const IconFit = (): ReactElement => (
  <Svg>
    <path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5" />
    <rect x="8.5" y="8.5" width="7" height="7" rx="1" />
  </Svg>
);

export const IconVehicle = (): ReactElement => (
  <Svg>
    <path d="M12 3l7 17-7-4-7 4z" />
  </Svg>
);

export const IconFollow = (): ReactElement => (
  <Svg>
    <circle cx="12" cy="12" r="3" />
    <path d="M12 2v4M12 18v4M2 12h4M18 12h4" />
  </Svg>
);

export const IconFolder = (): ReactElement => (
  <Svg>
    <path d="M3 7a2 2 0 012-2h4l2 2h8a2 2 0 012 2v8a2 2 0 01-2 2H5a2 2 0 01-2-2z" />
  </Svg>
);

export const IconSave = (): ReactElement => (
  <Svg>
    <path d="M5 4h11l3 3v12a1 1 0 01-1 1H6a1 1 0 01-1-1z" />
    <path d="M8 4v5h7V4M8 20v-6h8v6" />
  </Svg>
);

export const IconPlus = (): ReactElement => (
  <Svg>
    <path d="M12 5v14M5 12h14" />
  </Svg>
);

export const IconChevron = (): ReactElement => (
  <Svg>
    <path d="M7 10l5 5 5-5" />
  </Svg>
);
